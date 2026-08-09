from __future__ import annotations

import numpy as np
import torch

from Pose import Pose, PoseDelta
from Skeleton import Skeleton
from motion_field_io import (DELTA_YAW_CONVENTION, TUG_MODE, ValueFunctionData,
                             database_signature, load_value_function)

# Packed pose layout: row 0 root position, row 1 hip position, row 2 the root
# bone quaternion, rows 3+ the remaining joint quaternions.
ROOT_QUAT_ROW = 2

_TWO_PI = 2.0 * np.pi


def wrap_angle(angle):
    """Wrap radians into [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % _TWO_PI - np.pi


def root_yaw(packed_x: np.ndarray) -> np.ndarray:
    """
    Yaw carried by the root quaternion of a packed pose, in radians.

    Exact rather than approximate: PoseExtractor builds the SimulationBone
    rotation as LookRotation(hipsForward_projected, up), so every root rotation
    in this system is a pure yaw and reduces to 2*atan2(q.y, q.w). Note the
    xyzw component order -- the wxyz formula the reference implementation uses
    would read x and z here and return nonsense.
    """
    q = np.asarray(packed_x)[..., ROOT_QUAT_ROW, :]
    return 2.0 * np.arctan2(q[..., 1], q[..., 3])


def resolve_device(preference=None) -> str:
    if preference in (None, '', 'auto'):
        return 'cuda' if torch.cuda.is_available() else 'cpu'
    if preference == 'cuda' and not torch.cuda.is_available():
        print('[MotionField] CUDA requested but unavailable; falling back to CPU.')
        return 'cpu'
    return preference


class MotionField:
    def __init__(self,
                 poses_x: np.ndarray,
                 poses_v: np.ndarray, poses_y: np.ndarray,
                 skeleton: Skeleton,
                 frame_time: float,
                 device=None,
                 pos_weight: float = 0.2,
                 vel_weight: float = 0.9,
                 k_neighbors: int = 15,
                 tug_ratio: float = 0.1,
                 knn_chunk: int = 64,
                 value_function_path: str = None):
        """
        Initialize the MotionField.

        :param poses_x: Pose in array format (root, hips, quats) for each state. Shape: (state_count, num_bones + 2, 4)
        :param poses_v: Pose velocity in array format (root vel, hips vel, rotvecs) for each state, per second. Shape: (state_count, num_bones + 2, 4)
        :param poses_y: Next pose velocity in array format (root vel, hips vel, rotvecs) for each state, per second. Shape: (state_count, num_bones + 2, 4)
        :param skeleton: The Skeleton object used for forward kinematics of the poses.
        :param frame_time: Seconds per frame of the source database. Velocities are per-second rates, so this is needed to turn them into a one-frame step.
        :param device: 'cuda', 'cpu' or None/'auto' to pick whatever is present.
        :param pos_weight: Weight on the position half of the similarity feature.
        :param vel_weight: Weight on the velocity half of the similarity feature.
        :param k_neighbors: Default neighbourhood size for k-NN and for the action set.
        :param tug_ratio: Drift correction strength toward the database, per step.
        :param knn_chunk: Queries per batched k-NN chunk. Bounds VRAM at roughly
            chunk * state_count * feature_count * 3 * 4 bytes.
        :param value_function_path: Optional `.mffield.npz` enabling `optimal_action`.
        """
        self.current_frame = None
        assert (poses_x.shape[:-2] ==
                poses_v.shape[:-2] ==
                poses_y.shape[:-2]), "Inconsistent batch shapes"

        self.frame_time = frame_time
        self.pos_weight = pos_weight
        self.vel_weight = vel_weight
        self.k_neighbors = k_neighbors
        self.tug_ratio = tug_ratio
        self.knn_chunk = max(1, int(knn_chunk))
        self.device = resolve_device(device)

        self.poses_x = poses_x
        self.poses_v = poses_v
        self.poses_y = poses_y
        self.skeleton = skeleton
        self.states_count = poses_x.shape[0]
        self.bone_count = poses_x.shape[-2] - 2

        self.state_features = self.features(poses_x, poses_v)
        self.feature_shape = self.state_features.shape[-2:]
        self.states = torch.from_numpy(self.state_features).to(self.device)

        self.value_function: ValueFunctionData | None = None
        if value_function_path:
            self.load_value_function(value_function_path)

    # ------------------------------------------------------------------ #
    # Similarity features and nearest-neighbour search
    # ------------------------------------------------------------------ #

    def features(self, x: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Similarity feature for one or many packed states."""
        return MotionField.build_motion_states(
            x, v, self.skeleton, self.frame_time,
            pos_weight=self.pos_weight, vel_weight=self.vel_weight)

    def get_batched_knn(self, queries: np.ndarray, k: int = None, chunk: int = None):
        """
        Brute-force k-NN for many queries at once.

        :param queries: (query_count, feature_count, 3)
        :return: (indices (query_count, k) int64, weights (query_count, k) float32)
            where the weights are the normalised inverse-square distances the
            paper calls similarity weights.
        """
        k = self.k_neighbors if k is None else k
        chunk = self.knn_chunk if chunk is None else max(1, int(chunk))

        queries = np.ascontiguousarray(queries, dtype=np.float32)
        if queries.ndim == 2:
            queries = queries[np.newaxis, ...]

        query_tensor = torch.from_numpy(queries).to(self.device)
        states = self.states.unsqueeze(0)  # (1, state_count, feature_count, 3)

        indices = np.empty((queries.shape[0], k), dtype=np.int64)
        distances = np.empty((queries.shape[0], k), dtype=np.float32)

        # Chunked because the difference tensor is
        # chunk * state_count * feature_count * 3 floats -- a few hundred MB at
        # chunk=64 on this database, and quadratic in the chunk size.
        for start in range(0, query_tensor.shape[0], chunk):
            q = query_tensor[start:start + chunk].unsqueeze(1)  # (c, 1, F, 3)
            diff = states - q                                   # (c, S, F, 3)
            per_joint = torch.norm(diff, dim=3)                 # (c, S, F)
            summed = torch.sum(per_joint, dim=2)                # (c, S)
            top_k = torch.topk(summed, k=k, largest=False)
            indices[start:start + chunk] = top_k.indices.cpu().numpy()
            distances[start:start + chunk] = top_k.values.cpu().numpy()

        return indices, MotionField.calculate_similarity_weights(distances)

    def get_knn(self, motion_state, k):
        """Single-query k-NN. Returns (indices (k,), distances (k,))."""
        query = np.ascontiguousarray(motion_state, dtype=np.float32)
        if query.ndim == 3:
            query = query[0]

        query_tensor = torch.from_numpy(query).to(self.device)

        diff = self.states - query_tensor      # (state_count, feature_count, 3)
        point_distances = torch.norm(diff, dim=2)
        sum_distances = torch.sum(point_distances, dim=1)

        top_k = torch.topk(sum_distances, k=k, largest=False)
        return top_k.indices.cpu().numpy(), top_k.values.cpu().numpy()

    @staticmethod
    def calculate_similarity_weights(distances: np.ndarray) -> np.ndarray:
        """Normalised inverse-square distance weights, over the last axis."""
        weights = 1.0 / (np.asarray(distances, dtype=np.float32) ** 2 + 1e-8)
        return (weights / np.sum(weights, axis=-1, keepdims=True)).astype(np.float32)

    @staticmethod
    def build_action_weights(weights: np.ndarray) -> np.ndarray:
        """
        The candidate action set for one state.

        One action per neighbour: take the similarity weights and pin that
        neighbour's weight to 1 before renormalising, so each action leans on a
        different neighbour while still respecting the similarity distribution.
        Action i is row i.

        :param weights: (k,) similarity weights
        :return: (k, k) float32, each row a convex combination
        """
        k = weights.shape[-1]
        actions = np.tile(np.asarray(weights, dtype=np.float32), (k, 1))
        actions[np.arange(k), np.arange(k)] = 1.0
        return actions / np.sum(actions, axis=1, keepdims=True)

    # ------------------------------------------------------------------ #
    # Integration
    # ------------------------------------------------------------------ #

    def compute_new_states(self,
                           current_pose_x: np.ndarray,
                           delta_time: float,
                           indices: np.ndarray,
                           action_weights: np.ndarray,
                           tug_indices: np.ndarray,
                           tug_ratio: float = None):
        """
        Integrate several candidate actions from a pose, in one batch.

        :param current_pose_x: (num_bones+2, 4), (1, num_bones+2, 4) or
            (action_count, num_bones+2, 4) if each action starts somewhere else
        :param indices: (k,) neighbour state ids shared by every action, or
            (action_count, k) when each action has its own neighbourhood --
            which is what the trainer needs to evaluate every database state at
            once.
        :param action_weights: (action_count, k) convex weights per action
        :param tug_indices: (action_count,) database state each action is tugged toward
        :return: (x, v), both (action_count, num_bones+2, 4)
        """
        tug_ratio = self.tug_ratio if tug_ratio is None else tug_ratio
        action_weights = np.ascontiguousarray(action_weights, dtype=np.float32)
        action_count = action_weights.shape[0]
        tug_indices = np.asarray(tug_indices).reshape(-1)

        x0 = np.asarray(current_pose_x, dtype=np.float32)
        if x0.ndim == 2:
            x0 = x0[np.newaxis, ...]
        if x0.shape[0] == 1 and action_count > 1:
            x0 = np.broadcast_to(x0, (action_count,) + x0.shape[1:])
        assert x0.shape[0] == action_count, \
            f'expected {action_count} start poses, got {x0.shape[0]}'

        current = Pose.from_array(x0)  # from_array copies, so this is writable

        # The root slot is a PER-FRAME INCREMENT, not a world pose. Database
        # states are stored root-relative (rootPos == 0, root quat == identity,
        # see action_predictor.load_animations) and the tug below blends against
        # one of them, so an accumulated world root would be dragged toward the
        # origin by `tug_ratio` every frame -- a 0.9^n collapse that reaches the
        # origin in about 30 frames. Unity owns the world root anyway and
        # integrates it from the returned velocities
        # (MotionSynthesisComponent.ApplyPoseToSkeletonTransforms), so zeroing
        # here also makes training and runtime structurally identical: the root
        # of the returned pose is exactly the increment this step produced.
        current.rootPos[...] = 0.0
        current.quats[..., 0, :] = (0.0, 0.0, 0.0, 1.0)

        # Blend the neighbour velocities under each action's weights.
        indices = np.asarray(indices)
        gather = 'kbc,ak->abc' if indices.ndim == 1 else 'akbc,ak->abc'
        blended_v = np.einsum(gather, self.poses_v[indices, ...], action_weights)
        blended_y = np.einsum(gather, self.poses_y[indices, ...], action_weights)

        next_pose_x = current.add(PoseDelta.from_array(blended_v).scaled(delta_time))
        next_pose_v = PoseDelta.from_array(blended_y)

        # Drift correction: pull a little toward an actual database state so the
        # field cannot wander into regions it has no data for.
        nearest_pose_x = Pose.from_array(self.poses_x[tug_indices]).add(
            PoseDelta.from_array(self.poses_v[tug_indices]).scaled(delta_time))
        nearest_pose_v = PoseDelta.from_array(self.poses_y[tug_indices])

        tug_weights = np.array([1.0 - tug_ratio, tug_ratio], dtype=np.float32)
        tugged_pose_x = Pose.blend(Pose.stack([next_pose_x, nearest_pose_x]), tug_weights)
        tugged_pose_v = PoseDelta.blend(
            PoseDelta.stack([next_pose_v, nearest_pose_v]), tug_weights)

        return tugged_pose_x.pack(), tugged_pose_v.pack()

    def compute_new_state(self,
                          current_pose_x: np.ndarray,
                          delta_time: float,
                          indices,
                          weights,
                          nearest_pose_index=None,
                          tug_ratio: float = None):
        """
        Integrate a single action. Returns (x, v), both (1, num_bones+2, 4).

        By default the tug target is the neighbour the action emphasises, which
        is what makes the candidate actions meaningfully different from each
        other; tugging everything toward the nearest state instead collapses
        them onto near-identical outcomes.
        """
        weights = np.asarray(weights, dtype=np.float32)
        tug = indices[int(np.argmax(weights))] if nearest_pose_index is None else nearest_pose_index
        return self.compute_new_states(current_pose_x, delta_time, indices,
                                       weights[np.newaxis, :], np.asarray([tug]),
                                       tug_ratio)

    # ------------------------------------------------------------------ #
    # Value function
    # ------------------------------------------------------------------ #

    def load_value_function(self, path: str, data_dir: str = None,
                            db_name: str = None) -> bool:
        """
        Load a trained value function, gating it against this field's own
        parameters. Returns False (and leaves the field in greedy mode) when the
        file is missing or stale rather than raising -- Unity calls this inside
        Py.GIL() where an exception surfaces as an opaque managed error.

        Pass `data_dir`/`db_name` to additionally verify that the file was
        trained on exactly the pose database now loaded. Every index in the
        value function is a row of that database, so a regenerated `.mmpose`
        with the same frame count would otherwise address the wrong poses.
        """
        expect = {
            'delta_yaw_convention': DELTA_YAW_CONVENTION,
            'states_count': self.states_count,
            'bone_count': self.bone_count,
            'k_neighbors': self.k_neighbors,
            'tug_ratio': self.tug_ratio,
            'tug_mode': TUG_MODE,
            'pos_weight': self.pos_weight,
            'vel_weight': self.vel_weight,
            'frame_time': self.frame_time,
        }
        if data_dir and db_name:
            try:
                expect.update(database_signature(data_dir, db_name))
            except OSError as exc:
                print(f'[MotionField] could not hash the pose database: {exc}')

        self.value_function = load_value_function(path, expect=expect)
        if self.value_function is not None:
            print(f'[MotionField] loaded value function {path} '
                  f'({self.value_function.values.shape[0]} states x '
                  f'{self.value_function.theta_count} headings, '
                  f'final residual {self.value_function.final_residual:.6f})')
        return self.value_function is not None

    @property
    def has_value_function(self) -> bool:
        return self.value_function is not None

    def interpolate_value(self, indices: np.ndarray, weights: np.ndarray,
                          theta: np.ndarray) -> np.ndarray:
        """
        V(s', theta') by bilinear interpolation.

        Linear across the two bracketing headings on the task grid, then
        weighted by the motion-space similarity weights. Sampling the value
        function at only the discrete database states is what makes it behave as
        if it were continuous over the whole field.

        :param indices: (n, k) neighbour state ids
        :param weights: (n, k) similarity weights
        :param theta: (n,) goal headings
        :return: (n,)
        """
        vf = self.value_function
        coord = (np.asarray(theta, dtype=np.float32) + np.pi) / vf.theta_spacing
        lower = np.floor(coord)
        alpha = (coord - lower)[:, np.newaxis]
        i0 = (lower.astype(np.int64) % vf.theta_count)[:, np.newaxis]
        i1 = (i0 + 1) % vf.theta_count

        v = (1.0 - alpha) * vf.values[indices, i0] + alpha * vf.values[indices, i1]
        return np.sum(v * weights, axis=1)

    def value_reward(self, theta: float, next_x: np.ndarray, next_v: np.ndarray) -> float:
        """
        Reward for landing in a state, using the trained value function.

        This is the Bellman objective the training optimised:
        `Q = R(s, a) + gamma * V(s')`. The immediate term keeps the runtime
        consistent with the value iteration -- scoring on the discounted future
        value alone would optimise something the value function was never fitted
        to.

        :param theta: the character's current heading in the goal frame, radians
        :param next_x: packed pose after the action, (num_bones+2, 4) or (1, ...)
        :param next_v: matching packed velocity
        :return: scalar Q
        """
        return float(self.value_rewards(theta, next_x, next_v)[0])

    def value_rewards(self, theta: float, next_x: np.ndarray, next_v: np.ndarray) -> np.ndarray:
        """Batched `value_reward` over several candidate outcomes. Returns (n,)."""
        assert self.has_value_function, 'no value function loaded'

        next_x = np.asarray(next_x, dtype=np.float32)
        next_v = np.asarray(next_v, dtype=np.float32)
        if next_x.ndim == 2:
            next_x = next_x[np.newaxis, ...]
            next_v = next_v[np.newaxis, ...]

        # Each action rotates the character, so the goal moves in its frame.
        theta_prime = wrap_angle(theta + root_yaw(next_x)).astype(np.float32)

        indices, weights = self.get_batched_knn(self.features(next_x, next_v),
                                                self.value_function.k_neighbors)
        future = self.interpolate_value(indices, weights, theta_prime)
        return -np.abs(theta_prime) + self.value_function.gamma * future

    # ------------------------------------------------------------------ #
    # Policies
    # ------------------------------------------------------------------ #

    def optimal_action(self, theta: float,
                       current_x: np.ndarray, current_v: np.ndarray,
                       delta_time: float, k_neighbors: int = None):
        """
        Step the field along the action with the highest long-term value.

        Unlike `greedy_action`, which only asks which action points the
        character closest to the goal on the very next frame, this scores each
        candidate with `R + gamma * V(s')` and so will accept a worse immediate
        heading when it sets up a better turn. That anticipation is the whole
        reason for training a value function.

        :param theta: the character's current heading expressed in the goal
            frame, radians. Unity computes it as
            `-SignedAngle(root.forward, desiredWorldDir, up)`.
        :return: (new_x, new_v), both (1, num_bones+2, 4)
        """
        if not self.has_value_function:
            return self.greedy_action(theta, current_x, current_v, delta_time, k_neighbors)

        k = k_neighbors or self.value_function.k_neighbors
        indices, distances = self.get_knn(self.features(current_x, current_v), k=k)
        weights = MotionField.calculate_similarity_weights(distances)

        actions = MotionField.build_action_weights(weights)
        xs, vs = self.compute_new_states(current_x, delta_time, indices, actions,
                                         tug_indices=indices)

        best = int(np.argmax(self.value_rewards(theta, xs, vs)))
        return xs[best:best + 1], vs[best:best + 1]

    def greedy_action(self, theta: float,
                      current_x: np.ndarray, current_v: np.ndarray,
                      delta_time: float, k_neighbors: int = None):
        """
        One-step-greedy fallback: pick whichever action best aligns the heading
        with the goal on the next frame. Used when no value function is loaded.
        """
        k = k_neighbors or self.k_neighbors
        indices, distances = self.get_knn(self.features(current_x, current_v), k=k)
        weights = MotionField.calculate_similarity_weights(distances)

        actions = MotionField.build_action_weights(weights)
        xs, vs = self.compute_new_states(current_x, delta_time, indices, actions,
                                         tug_indices=indices)

        rewards = -np.abs(wrap_angle(theta + root_yaw(xs)))
        best = int(np.argmax(rewards))
        return xs[best:best + 1], vs[best:best + 1]

    def get_next_pose(self, current_x: np.ndarray, current_v: np.ndarray, delta_time):
        """Debug playback: walk the database frame by frame from the nearest match."""
        if self.current_frame is None:
            indices, _ = self.get_knn(self.features(current_x, current_v), k=self.k_neighbors)
            self.current_frame = int(indices[0])
        else:
            self.current_frame = (self.current_frame + 1) % self.states_count

        return (self.poses_x[self.current_frame][np.newaxis, ...],
                self.poses_v[self.current_frame][np.newaxis, ...])

    def get_next_pose_from_field(self, current_x: np.ndarray, current_v: np.ndarray, delta_time):
        """Debug: snap to the successor of the nearest database state."""
        indices, _ = self.get_knn(self.features(current_x, current_v), k=self.k_neighbors)
        nxt = min(int(indices[0]) + 1, self.states_count - 1)
        return self.poses_x[nxt][np.newaxis, ...], self.poses_v[nxt][np.newaxis, ...]

    # Selectable from Unity via MotionFieldStage.
    MODE_OPTIMAL = 'optimal'
    MODE_GREEDY = 'greedy'
    MODE_PLAYBACK = 'playback'
    MODE_NEAREST = 'nearest'

    def get_pose(self, current_x: np.ndarray, current_v: np.ndarray, delta_time,
                 theta: float = 0.0, mode: str = None):
        """
        Advance one step. Defaults to the value-function policy when one is
        loaded and degrades to greedy control when it is not.
        """
        mode = mode or (MotionField.MODE_OPTIMAL if self.has_value_function
                        else MotionField.MODE_GREEDY)

        if mode == MotionField.MODE_OPTIMAL:
            return self.optimal_action(theta, current_x, current_v, delta_time)
        if mode == MotionField.MODE_GREEDY:
            return self.greedy_action(theta, current_x, current_v, delta_time)
        if mode == MotionField.MODE_PLAYBACK:
            return self.get_next_pose(current_x, current_v, delta_time)
        if mode == MotionField.MODE_NEAREST:
            return self.get_next_pose_from_field(current_x, current_v, delta_time)
        raise ValueError(f'unknown motion field mode: {mode!r}')

    @staticmethod
    def build_motion_states(x, v, skeleton: Skeleton, frame_time: float,
                            pos_weight=0.2, vel_weight=0.9):
        """
        Build motion states from pose and velocity. Used to construct the motion field.

        The feature is the root-space joint positions concatenated with the
        root-space joint velocities. `v` holds per-second rates, so it must be
        scaled by `frame_time` to obtain a one-frame lookahead pose before the
        difference is taken; dividing back by `frame_time` leaves the velocity
        block in m/s so the two halves are dimensionally distinct and can be
        weighted independently.

        Root-invariant by construction: `fk_root_space` forces joint 0 to the
        origin with identity rotation, so where the character is in the world
        never influences which states it matches.

        :param x: Packed poses, shape (..., bone_count + 2, 4)
        :param v: Packed per-second velocities, shape (..., bone_count + 2, 4)
        :param skeleton: Skeleton used for forward kinematics
        :param frame_time: Seconds per frame of the source database
        :param pos_weight: Weight on the position half of the feature
        :param vel_weight: Weight on the velocity half of the feature
        :return: (..., bone_count * 2, 3)
        """
        current_pose = Pose.from_array(x)

        p_a, _ = skeleton.fk_root_space(current_pose)

        next_pose = current_pose.add(PoseDelta.from_array(v).scaled(frame_time))

        p_b, _ = skeleton.fk_root_space(next_pose)

        velocity = (p_b - p_a) / frame_time  # m/s

        return np.concatenate(
            [pos_weight * p_a, vel_weight * velocity],
            axis=-2).astype(np.float32)  # (batch, bone_count*2, 3)
