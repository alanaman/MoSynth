import torch

class MotionField:
    def __init__(self, motion_states):
        self.states = torch.from_numpy(motion_states).to('cuda').unsqueeze(0)

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