import numpy as np
import torch
from pxr.Gf._gf import Rotation

from Pose import Pose
from labs.Theory.laplacian_deformation import indices


class MotionField:
    def __init__(self,
                 motion_states: np.ndarray, poses_x: np.ndarray,
                 poses_v: np.ndarray, poses_y: np.ndarray):
        """
        Initialize the MotionField.
        :param motion_states: Holds global space positions and velocities.
        Shape: (state_count, feature_count, 3)
        :param poses_x: Pose in array format (root, hips, quats) for each state.
        Shape: (state_count, num_bones + 2, 4)
        :param poses_v: Pose velocity in array format (root, hips, quats) for each state.
        Shape: (state_count, num_bones + 2, 4)
        :param poses_y: Next pose velocity in array format (root, hips, quats) for each state.
        Shape: (state_count, num_bones + 2, 4)
        """
        assert (motion_states.shape[:-2] ==
                poses_x.shape[:-2] ==
                poses_v.shape[:-2] ==
                poses_y.shape[:-2]), "Inconsistent batch shapes"

        self.states = torch.from_numpy(motion_states).to('cuda').unsqueeze(0)
        self.poses_x = poses_x
        self.poses_v = poses_v
        self.poses_y = poses_y

    def get_knn(self, motion_state, k):
        query = torch.from_numpy(motion_state).to('cuda')

        # Expand p and q to broadcast
        p_exp = self.states  # shape: (1, state_count, feature_count, 3)
        q_exp = query.unsqueeze(1)  # shape: (query_count, 1, feature_count, 3)

        diff = p_exp - q_exp  # shape: (query_count, state_count, feature_count, 3), broadcasting
        point_distances = torch.norm(diff, dim=3)  # shape: (query_count, state_count, feature_count)

        # Step 2: sum distances per set
        sum_distances = torch.sum(point_distances, dim=2)  # shape: (query_count, state_count)

        # Step 3: get k nearest sets for each query
        top_k = torch.topk(sum_distances, k=k, largest=False)
        knn_indices = top_k.indices.cpu().numpy()  # shape: (query_count, k)
        knn_distances = top_k.values.cpu().numpy()  # shape: (query_count, k)

        return knn_indices, knn_distances

    def compute_new_state(self, current_pose_x: np.ndarray, indices, weights, tug_ratio=0.1):

        blended_v = Pose.blend(Pose.from_array(self.poses_v[indices, ...]), weights)
        blended_y = Pose.blend(Pose.from_array(self.poses_y[indices, ...]), weights)

        tug_index = indices[np.argmax(weights)]
        nearest_pose_x = Pose.from_array(self.poses_x[tug_index])
        nearest_pose_v = Pose.from_array(self.poses_v[tug_index])
        nearest_pose_y = Pose.from_array(self.poses_y[tug_index])
        current_pose_x = Pose.from_array(current_pose_x)

        nearest_pose_x.rootPos = current_pose_x.rootPos
        nearest_pose_x.quats[0] = current_pose_x.quats[0]

        tug_v = nearest_pose_x + nearest_pose_v - current_pose_x
        tug_y = nearest_pose_y

        tugged_v = Pose.blend(Pose.concatenate([blended_v, tug_v]), np.array([1-tug_ratio, tug_ratio]))
        tugged_y = Pose.blend(Pose.concatenate([blended_y, tug_y]), np.array([1-tug_ratio, tug_ratio]))

        return (current_pose_x + tugged_v).pack(), tugged_y.pack()

    @staticmethod
    def calculate_similarity_weights(distances: np.ndarray) -> np.ndarray:
        weights = 1.0 / (distances**2 + 1e-8)
        weights = weights / np.sum(weights)
        return weights
