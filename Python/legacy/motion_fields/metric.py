"""Similarity metric between motion states.

Unlike the paper (which compares joint rotations), this implementation compares
world-space bone positions of the current pose and of the pose one step ahead,
which makes distances directly interpretable and cheap to evaluate in bulk.

``init_metric`` must be called once, after :func:`motion_fields.pose.init_pose_module`.
"""

import numpy as np
import ipyanimlab as lab

from .pose import pose_add, pose_pack, pose_unpack

FEATURE_SHAPE = None

_parents = None
_default_skeleton_p = None
_metric_weights = None


def init_metric(bone_count, parents, default_skeleton_p, metric_weights):
    global FEATURE_SHAPE, _parents, _default_skeleton_p, _metric_weights
    FEATURE_SHAPE = (bone_count * 2, 3)
    _parents = parents
    _default_skeleton_p = np.asarray(default_skeleton_p, dtype=np.float32)
    _metric_weights = np.asarray(metric_weights, dtype=np.float32)


def build_distance_metric(x, v):
    """Feature vector for a motion state: weighted positions + one-step deltas."""
    next_pose = pose_unpack(x.copy())
    next_pose.quats[0] = [1, 0, 0, 0]
    _, p_a = lab.utils.quat_fk(next_pose.quats, _default_skeleton_p, _parents)

    next_pose = pose_unpack(
        pose_add(pose_pack(next_pose.root, next_pose.hips, next_pose.quats), v))

    _, p_b = lab.utils.quat_fk(next_pose.quats, _default_skeleton_p, _parents)

    return np.concatenate(
        [p_a * _metric_weights[:, np.newaxis] * .8, p_b - p_a]).astype(np.float32)


def build_metric_matrix(states_x, states_v):
    """Feature vectors for the whole state database: (states, bones*2, 3)."""
    count = states_x.shape[0]
    matrix = np.zeros([count, FEATURE_SHAPE[0], 3], dtype=np.float32)
    for i in range(count):
        matrix[i, ...] = build_distance_metric(states_x[i], states_v[i])
    return matrix
