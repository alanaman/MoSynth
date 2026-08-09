"""Motion field integration.

An action is a convex combination over the k nearest neighbours. Integrating it
blends the neighbours' velocities to advance the pose, and blends their "next"
velocities to produce the new state velocity.

A small drift correction ("tug") pulls the result toward the single most similar
database state, which keeps the character from wandering into regions the motion
data does not cover.
"""

import numpy as np
import ipyanimlab as lab

from .pose import (pose_add, pose_blend, pose_lerp, pose_pack, pose_subtract,
                   pose_unpack)


def compute_v_to_reach_state(states_x, states_v, current_x, state_id):
    """Velocity that would take `current_x` onto the successor of `state_id`."""
    next_x = pose_add(states_x[state_id], states_v[state_id])
    next_x = pose_unpack(next_x)
    x = pose_unpack(current_x)
    next_x.root += x.root
    next_x.quats[0] = lab.utils.quat_mul(x.quats[0], next_x.quats[0])
    next_x = pose_pack(next_x.root, next_x.hips, next_x.quats)
    return pose_subtract(next_x, current_x)


def compute_new_state(states_x, states_v, states_y, current_x, indices, weights,
                      tug_ratio=.1):
    """Integrate one frame. Returns the new pose and its velocity."""
    tug_indice = np.argmax(weights)
    next_v = compute_v_to_reach_state(states_x, states_v, current_x, indices[tug_indice])

    blended_v = pose_blend(states_v[indices, ...], weights)
    final_v = pose_lerp(blended_v, next_v, tug_ratio)

    blended_y = pose_blend(states_y[indices, ...], weights)
    final_y = pose_lerp(blended_y, states_y[indices[tug_indice]], tug_ratio)

    return pose_add(current_x, final_v), final_y


def signed_angle(desired, current, up=np.array([0, 1, 0])):
    """Ground-plane angle from `current` to `desired`, signed about `up`."""
    d = desired.copy()
    d[1] = 0
    d /= np.linalg.norm(d) + 1e-12
    c = current.copy()
    c[1] = 0
    c /= np.linalg.norm(c) + 1e-12

    dot = np.clip(np.dot(d, c), -1.0, 1.0)
    angle = np.arccos(dot)

    cross = np.cross(c, d)
    sign = np.sign(np.dot(cross, up))
    return angle * sign


def action_reward(desired_direction, current_x, next_x):
    """Immediate reward: how well the next pose faces the desired direction."""
    n = pose_unpack(next_x)
    orientation = lab.utils.quat_mul_vec(n.quats[0], np.array([0, 0, 1], dtype=np.float32))
    return -abs(signed_angle(desired_direction, orientation))


def compute_theta_blend_factors(theta_prime, theta_count, theta_spacing):
    """Locate `theta_prime` on the discretised goal-angle grid."""
    lower_id = int((theta_prime + np.pi) / theta_spacing) % theta_count
    upper_id = (lower_id + 1) % theta_count
    theta_lower = theta_spacing * lower_id - np.pi
    angle_offset = (theta_prime - theta_lower + 2 * np.pi) % (2 * np.pi)
    t = angle_offset / theta_spacing
    return lower_id, upper_id, t
