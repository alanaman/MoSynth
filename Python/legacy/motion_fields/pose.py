"""Pose algebra.

A pose is packed into a single ``(bone_count + 2, 4)`` float32 array::

    row 0    -> root position   (xyz in [:3])
    row 1    -> hips position   (xyz in [:3])
    row 2..  -> joint quaternions (wxyz)

A velocity uses the same layout, holding the finite difference ``x' - x`` for
the positions and ``p' * p^-1`` for the rotations.

``init_pose_module`` must be called once before any other function here; it
supplies the skeleton dimensions the notebook kept as globals.
"""

import numpy as np
import ipyanimlab as lab

POSESHAPE = None
default_skeleton_p = None


def init_pose_module(bone_count, skeleton_p):
    """Bind the skeleton this module operates on."""
    global POSESHAPE, default_skeleton_p
    POSESHAPE = (bone_count + 2, 4)
    default_skeleton_p = np.asarray(skeleton_p, dtype=np.float32)


class PoseData:
    def __init__(self, root, hips, quats):
        self.root = root
        self.hips = hips
        self.quats = quats


def pose_pack(root, hips, quats):
    result = np.zeros(POSESHAPE, dtype=np.float32)
    result[0, :3] = root
    result[1, :3] = hips
    result[2:, :] = quats
    return result


def pose_unpack(pose):
    # Note: these are views into `pose`, not copies -- callers rely on that.
    root = pose[..., 0, :3]
    hips = pose[..., 1, :3]
    quats = pose[..., 2:, :]
    return PoseData(root, hips, quats)


def pose_add(x, v):
    x = pose_unpack(x)
    v = pose_unpack(v)

    _, root = lab.utils.qp_mul((x.quats[0], x.root), (v.quats[0], v.root))
    hips = x.hips + v.hips
    quats = lab.utils.normalize(lab.utils.quat_mul(x.quats, v.quats))

    return pose_pack(root, hips, quats)


def pose_subtract(a, b):
    a = pose_unpack(a)
    b = pose_unpack(b)

    _, root = lab.utils.qp_mul(lab.utils.qp_inv((b.quats[0], b.root)), (a.quats[0], a.root))
    hips = a.hips - b.hips
    quats = lab.utils.normalize(lab.utils.quat_mul(lab.utils.quat_inv(b.quats), a.quats))
    flip = quats[:, 0] < 0
    quats[flip, :] = -quats[flip, :]

    return pose_pack(root, hips, quats)


def pose_lerp(a, b, t):
    a = pose_unpack(a)
    b = pose_unpack(b)

    root = (1.0 - t) * a.root + t * b.root
    hips = (1.0 - t) * a.hips + t * b.hips
    quats = lab.utils.normalize(lab.utils.quat_slerp(a.quats, b.quats, t))

    return pose_pack(root, hips, quats)


def pose_blend(states, weights):
    states = pose_unpack(states)

    root = np.sum(states.root * weights[:, np.newaxis], axis=0)
    hips = np.sum(states.hips * weights[:, np.newaxis], axis=0)
    quats = lab.utils.normalize(
        np.sum(states.quats * weights[:, np.newaxis, np.newaxis], axis=0))

    return pose_pack(root, hips, quats)


def pose_to_qp(a):
    """Expand a packed pose into (local quaternions, local positions)."""
    a = pose_unpack(a)
    p = default_skeleton_p.copy()
    p[0, :] = a.root
    p[1, :] = a.hips
    return a.quats.copy(), p
