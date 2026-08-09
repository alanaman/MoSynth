"""Brute-force k-nearest-neighbour search over the state database.

The database is small enough that an exhaustive GPU scan beats any index
structure, so we just broadcast every query against every state.
"""

import numpy as np
import torch

from .metric import build_distance_metric


def resolve_device(preference='auto'):
    if preference == 'cpu':
        return torch.device('cpu')
    if preference == 'cuda':
        return torch.device('cuda')
    if torch.cuda.is_available():
        return torch.device('cuda')
    print('  [knn] CUDA unavailable, falling back to CPU (this will be slow).')
    return torch.device('cpu')


class KnnIndex:
    """Holds the metric matrix on-device and answers k-NN queries against it."""

    def __init__(self, metric_matrix, device, query_chunk=256):
        self.device = device
        self.query_chunk = max(1, int(query_chunk))
        self.features = torch.from_numpy(
            np.ascontiguousarray(metric_matrix, dtype=np.float32)
        ).to(device).unsqueeze(0)          # (1, state_count, feature_count, 3)

    @property
    def state_count(self):
        return self.features.shape[1]

    def get_nns_by_vector(self, vector, k):
        """vector: (query_count, feature_count, 3) -> (indices, distances)."""
        query = torch.from_numpy(
            np.ascontiguousarray(vector, dtype=np.float32)).to(self.device)

        all_indices, all_distances = [], []
        # Chunked: shape diff after broadcasting is (chunk, states, features, 3), so
        # querying every state at once would allocate tens of gigabytes.
        for start in range(0, query.shape[0], self.query_chunk):
            q_exp = query[start:start + self.query_chunk].unsqueeze(1)

            diff = self.features - q_exp                  # (c, S, F, 3)
            point_distances = torch.norm(diff, dim=3)     # (c, S, F)
            sum_distances = torch.sum(point_distances, dim=2)  # (c, S)

            topk = torch.topk(sum_distances, k=k, largest=False)
            all_indices.append(topk.indices.cpu().numpy())
            all_distances.append(topk.values.cpu().numpy())

        return np.concatenate(all_indices, axis=0), np.concatenate(all_distances, axis=0)

    def get_batched_k_neighbors(self, metric_vector, k=15):
        """Similarity weights are inverse squared distance, normalised."""
        indices, distances = self.get_nns_by_vector(metric_vector, k)
        idistances = 1.0 / (distances ** 2 + 1e-8)
        idistances /= np.sum(idistances, axis=1, keepdims=True)
        return indices, idistances

    def get_k_neighbors(self, current_x, current_v, k=15):
        indices, idistances = self.get_batched_k_neighbors(
            build_distance_metric(current_x, current_v)[np.newaxis, ...], k)
        return indices[0], idistances[0]
