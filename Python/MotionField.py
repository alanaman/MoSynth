import numpy as np
import torch
from scipy.spatial.transform import Rotation

from Pose import Pose
from Skeleton import Skeleton


class MotionField:
    def __init__(self,
                 poses_x: np.ndarray,
                 poses_v: np.ndarray, poses_y: np.ndarray,
                 skeleton: Skeleton):
        """
        Initialize the MotionField.

        :param poses_x: Pose in array format (root, hips, quats) for each state. Shape: (state_count, num_bones + 2, 4)
        :param poses_v: Pose velocity in array format (root, hips, quats) for each state. Shape: (state_count, num_bones + 2, 4)
        :param poses_y: Next pose velocity in array format (root, hips, quats) for each state. Shape: (state_count, num_bones + 2, 4)
        :param skeleton: The Skeleton object used for forward kinematics of the poses.
        """
        assert (poses_x.shape[:-2] ==
                poses_v.shape[:-2] ==
                poses_y.shape[:-2]), "Inconsistent batch shapes"

        motion_states = MotionField.build_motion_states(poses_x, poses_v, skeleton)
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
                          tug_ratio=0.1):
        blended_v = Pose.blend(Pose.from_array(self.poses_v[indices, ...]), weights)
        blended_y = Pose.blend(Pose.from_array(self.poses_y[indices, ...]), weights)

        tug_index = indices[np.argmax(weights)]
        nearest_pose_x = Pose.from_array(self.poses_x[tug_index])
        nearest_pose_v = Pose.from_array(self.poses_v[tug_index])
        nearest_pose_y = Pose.from_array(self.poses_y[tug_index])
        current_pose_x = Pose.from_array(current_pose_x)

        tug_v = nearest_pose_x + nearest_pose_v - current_pose_x
        tug_v.rootPos[..., :] = 0
        tug_v.quats[..., 0, :] = Rotation.identity().as_quat()
        tug_y = nearest_pose_y

        tugged_v = Pose.blend(Pose.concatenate([blended_v, tug_v]), np.array([1 - tug_ratio, tug_ratio]))
        tugged_y = Pose.blend(Pose.concatenate([blended_y, tug_y]), np.array([1 - tug_ratio, tug_ratio]))

        return (current_pose_x + tugged_v.scaled(delta_time)).pack(), tugged_y.pack()

    @staticmethod
    def calculate_similarity_weights(distances: np.ndarray) -> np.ndarray:
        weights = 1.0 / (distances ** 2 + 1e-8)
        weights = weights / np.sum(weights)
        return weights

    def greedy_action(self, desired_direction, current_x, current_v, delta_time, k_neighbors=15):
        current_state = MotionField.build_motion_states(current_x, current_v, self.skeleton)
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
        new_x, new_v = self.compute_new_state(current_x, delta_time, indices, weights)

        return new_x, new_v

    @staticmethod
    def build_motion_states(x, v, skeleton):
        """
        Build motion states from pose and velocity. Used to construct the motion field.
        :param x:
        :param v:
        :param skeleton:
        :return:
        """
        metric_weights = np.array(
            [
                0, .3, .1, .1, .5, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .2, .5, 1, 1, .2, .5, 1, 1],
            dtype=np.float32)
        metric_velocity_weights = np.array(
            [1, .8, .5, .1, .1, .5, .1, 0, 0, 0, 0, 0, 0, 0, 0, 1.2, 1.5, 2, 0, 1.2, 1.5, 2, 0],
            dtype=np.float32)

        current_pose = Pose.from_array(x)

        p_a, _ = skeleton.fk_root_space(current_pose)

        # next_pose = current_pose + Pose.from_array(v)

        # p_b, _ = skeleton.fk_root_space(next_pose)
        p_v, _ = skeleton.fk_root_space(Pose.from_array(v))

        return np.concatenate([p_a * metric_weights[:, np.newaxis] * .8, p_v],
                              axis=-2)  # (batch, bone_count*2, 3)
