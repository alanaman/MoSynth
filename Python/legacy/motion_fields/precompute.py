"""Precomputed transition tables.

Value iteration needs, for every (state, action) pair, both the resulting motion
state and the k-NN interpolation weights used to read the value function at that
resulting state. None of that depends on the values themselves, so it is computed
once up front and cached on disk.

Table shapes, with S states and K neighbours:
    states_x / states_v   (S, K, bones+2, 4)
    value_indices         (S, K, K)  int16
    value_weights         (S, K, K)  float32
"""

import time
from dataclasses import dataclass

import numpy as np

from . import metric as metric_mod
from . import progress
from .metric import build_distance_metric
from .integration import compute_new_state


@dataclass
class Tables:
    states_x: np.ndarray
    states_v: np.ndarray
    value_indices: np.ndarray
    value_weights: np.ndarray


def build(dataset, knn, cfg):
    k = cfg.k_neighbors
    count = dataset.states_count
    bone_count = dataset.bone_count

    actions_x = np.zeros((count, k, bone_count + 2, 4), dtype=np.float32)
    actions_v = np.zeros((count, k, bone_count + 2, 4), dtype=np.float32)
    value_indices = np.zeros((count, k, k), dtype=np.int16)
    value_weights = np.zeros((count, k, k), dtype=np.float32)

    print('  finding neighbours of every state...')
    future_indices, future_weights = knn.get_batched_k_neighbors(dataset.metric_matrix, k)

    print('  integrating every action of every state...')
    batched_query = np.zeros([k, metric_mod.FEATURE_SHAPE[0], 3], dtype=np.float32)
    step = progress.interval(50, count)
    t0 = time.time()
    for i in range(count):
        if i % step == 0 or i == count - 1:
            progress.line(f'    state {i + 1} / {count}')

        indices, weights = future_indices[i], future_weights[i]

        for n_idx in range(k):
            # Each candidate action emphasises one neighbour, keeping the
            # similarity distribution for the rest.
            w = weights.copy()
            w[n_idx] = 1.0
            w /= np.sum(w)

            actions_x[i, n_idx, ...], actions_v[i, n_idx, ...] = compute_new_state(
                dataset.states_x, dataset.states_v, dataset.states_y,
                dataset.states_x[i], indices, w, cfg.tug_ratio_learning)
            batched_query[n_idx, ...] = build_distance_metric(
                actions_x[i, n_idx, ...], actions_v[i, n_idx, ...])

        # Where each of those outcomes lands in the database.
        value_indices[i, :, :], value_weights[i, :, :] = knn.get_batched_k_neighbors(
            batched_query, k)

    progress.end()
    print(f'    done in {time.time() - t0:.1f}s')
    return Tables(actions_x, actions_v, value_indices, value_weights)


def save(tables, path, signature):
    np.savez(path,
             states_x=tables.states_x,
             states_v=tables.states_v,
             value_indices=tables.value_indices,
             value_weights=tables.value_weights,
             config_json=signature)


def load(path, cfg, dataset):
    """Return cached Tables, or None if absent / stale / wrong shape."""
    if not path.exists():
        print(f'  cache miss: {path.name} does not exist')
        return None

    expected = (dataset.states_count, cfg.k_neighbors, dataset.bone_count + 2, 4)

    # np.load is lazy, so the signature and shape checks cost nothing.
    with np.load(path, allow_pickle=False) as data:
        if str(data['config_json']) != cfg.tables_signature():
            print(f'  cache stale: {path.name} was built with a different config')
            return None
        if data['states_x'].shape != expected:
            print(f'  cache stale: {path.name} has shape {data["states_x"].shape}, '
                  f'expected {expected}')
            return None

        return Tables(data['states_x'], data['states_v'],
                      data['value_indices'], data['value_weights'])


def load_or_build(cfg, dataset, knn, force=False):
    tables = None if force else load(cfg.tables_cache_path, cfg, dataset)

    if tables is None:
        tables = build(dataset, knn, cfg)
        print(f'  writing {cfg.tables_cache_path.name}...')
        save(tables, cfg.tables_cache_path, cfg.tables_signature())
    else:
        print(f'  loaded transition tables from {cfg.tables_cache_path.name}')

    return tables
