import numpy as np
import torch
from scipy.spatial.transform import Rotation

from Pose import Pose, PoseDelta
from Skeleton import Skeleton


class MotionField:
    def __init__(self,
                 poses_x: np.ndarray,
                 poses_v: np.ndarray, poses_y: np.ndarray,
                 skeleton: Skeleton,
                 frame_time: float):
        """
        Initialize the MotionField.

        :param poses_x: Pose in array format (root, hips, quats) for each state. Shape: (state_count, num_bones + 2, 4)
        :param poses_v: Pose velocity in array format (root vel, hips vel, rotvecs) for each state, per second. Shape: (state_count, num_bones + 2, 4)
        :param poses_y: Next pose velocity in array format (root vel, hips vel, rotvecs) for each state, per second. Shape: (state_count, num_bones + 2, 4)
        :param skeleton: The Skeleton object used for forward kinematics of the poses.
        :param frame_time: Seconds per frame of the source database. Velocities are per-second rates, so this is needed to turn them into a one-frame step.
        """
        self.current_frame = None
        assert (poses_x.shape[:-2] ==
                poses_v.shape[:-2] ==
                poses_y.shape[:-2]), "Inconsistent batch shapes"

        self.frame_time = frame_time
        motion_states = MotionField.build_motion_states(poses_x, poses_v, skeleton, frame_time)
        self.states = torch.from_numpy(motion_states).to('cuda')
        self.poses_x = poses_x
        self.poses_v = poses_v
        self.poses_y = poses_y
        self.skeleton = skeleton

    def get_knn(self, motion_state, k):
        query = torch.from_numpy(motion_state).to('cuda')

        # Expand p and q to broadcast
        p_exp = self.states  # shape: (state_count, feature_count, 3)
        q_exp = query  # shape: (1, feature_count, 3)

        diff = p_exp - q_exp  # shape: (state_count, feature_count, 3), broadcasting
        point_distances = torch.norm(diff, dim=2)  # shape: (state_count, feature_count)

        # Step 2: sum distances per set
        sum_distances = torch.sum(point_distances, dim=1)  # shape: (state_count,)

        # Step 3: get k nearest sets for each query
        top_k = torch.topk(sum_distances, k=k, largest=False)
        knn_indices = top_k.indices.cpu().numpy()  # shape: (k)
        knn_distances = top_k.values.cpu().numpy()  # shape: (k)

        return knn_indices, knn_distances

    def compute_new_state(self,
                          current_pose_x: np.ndarray,
                          delta_time: float,
                          indices,
                          weights,
                          nearest_pose_index=None,
                          tug_ratio=0.2):
        blended_v = PoseDelta.blend(PoseDelta.from_array(self.poses_v[indices, ...]), weights)
        blended_y = PoseDelta.blend(PoseDelta.from_array(self.poses_y[indices, ...]), weights)

        current_pose_x = Pose.from_array(current_pose_x)

        next_pose_x = current_pose_x.add(blended_v.scaled(delta_time))
        next_pose_v = blended_y

        tug_index = indices[np.argmax(weights)] if nearest_pose_index is None else nearest_pose_index
        nearest_pose_x = Pose.from_array(self.poses_x[tug_index]).add(
            PoseDelta.from_array(self.poses_v[tug_index]).scaled(delta_time))
        nearest_pose_v = PoseDelta.from_array(self.poses_y[tug_index])

        tugged_pose_x = Pose.blend(
            Pose.concatenate([next_pose_x, nearest_pose_x]),
            np.array([1 - tug_ratio, tug_ratio])
        )
        tugged_pose_v = PoseDelta.blend(
            PoseDelta.concatenate([next_pose_v, nearest_pose_v]),
            np.array([1 - tug_ratio, tug_ratio])
        )

        return tugged_pose_x.pack(), tugged_pose_v.pack()

    @staticmethod
    def calculate_similarity_weights(distances: np.ndarray) -> np.ndarray:
        weights = 1.0 / (distances ** 2 + 1e-8)
        weights = weights / np.sum(weights)
        return weights

    def greedy_action(self, desired_direction, current_x, current_v, delta_time, k_neighbors=15):
        current_state = MotionField.build_motion_states(current_x, current_v, self.skeleton, self.frame_time)
        indices, distances = self.get_knn(current_state, k=k_neighbors)
        weights = MotionField.calculate_similarity_weights(distances)

        rewards = np.zeros(k_neighbors)

        def get_reward(desired_vel, next_x):
            next_pose = Pose.from_array(next_x)
            next_root_dir_3d = np.array(
                Rotation.from_quat(next_pose.quats[0, 0, :]).apply(np.array([0, 0, 1]))
            )
            next_root_dir_2d = next_root_dir_3d[[0, 2]]  # Project to XZ plane
            return -np.linalg.norm(desired_vel - next_root_dir_2d)

        for n_idx in range(k_neighbors):
            w = weights.copy()
            w[n_idx] = 1.0
            w /= np.sum(w)
            nx, nv = self.compute_new_state(current_x, delta_time, indices, w)

            reward = get_reward(desired_direction, nx)

            # store the rewards
            rewards[n_idx] = reward

        best_i = np.argmax(rewards)
        weights[best_i] = 1.0
        weights /= np.sum(weights)
        new_x, new_v = self.compute_new_state(current_x, delta_time, indices, weights, tug_ratio=0.9)
        return new_x, new_v

    def get_next_pose_blended(self, current_x: np.ndarray, current_v: np.ndarray, delta_time, k_neighbors=15):
        current_state = MotionField.build_motion_states(current_x, current_v, self.skeleton, self.frame_time)
        indices, distances = self.get_knn(current_state, k=k_neighbors)
        weights = MotionField.calculate_similarity_weights(distances)

        best_i = 0
        weights[best_i] = 1.0
        weights /= np.sum(weights)
        new_x, new_v = self.compute_new_state(current_x, delta_time, indices, weights,
                                              nearest_pose_index=indices[best_i],
                                              tug_ratio=0.1)  # Use only the best neighbor for next pose

        return new_x, new_v

    def get_next_pose(self, current_x: np.ndarray, current_v: np.ndarray, delta_time):
        current_state = MotionField.build_motion_states(current_x, current_v, self.skeleton, self.frame_time)

        if self.current_frame is None:
            indices, distances = self.get_knn(current_state, k=15)
            self.current_frame = indices[0]
        else:
            self.current_frame += 1

        # weights = MotionField.calculate_similarity_weights(distances)
        #
        # print(indices[0], delta_time, flush=True)
        # new_x, new_v = self.compute_new_state(
        #     current_x,
        #     delta_time,
        #     indices[:1],
        #     np.array([1.0]),
        #     tug_ratio=1)  # Use only the best neighbor for next pose
        return self.poses_x[self.current_frame], self.poses_v[self.current_frame]

    def get_next_pose_from_field(self, current_x: np.ndarray, current_v: np.ndarray, delta_time):
        current_state = MotionField.build_motion_states(current_x, current_v, self.skeleton, self.frame_time)
        indices, distances = self.get_knn(current_state, k=15)
        weights = MotionField.calculate_similarity_weights(distances)

        return self.poses_x[indices[0] + 1], self.poses_v[indices[0] + 1]

    def get_pose(self, current_x: np.ndarray, current_v: np.ndarray, delta_time):
        return self.get_next_pose_blended(current_x, current_v, delta_time)
        return self.get_next_pose_from_field(current_x, current_v, delta_time)
        return self.get_next_pose(current_x, current_v, delta_time)
        return self.greedy_action(np.array([0.0, 0.0]), current_x, current_v, delta_time, k_neighbors=15)

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
            axis=-2)  # (batch, bone_count*2, 3)
