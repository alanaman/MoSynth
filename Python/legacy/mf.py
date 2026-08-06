import pickle
from dataclasses import dataclass, field
from random import randrange
import numpy as np
from ipywidgets import widgets, interact
from matplotlib import pyplot as plt

import torch
import torch.nn as nn

import ipyanimlab as lab

viewer = lab.Viewer(move_speed=5, width=1280, height=720)

# load the character
character = viewer.import_usd_asset('AnimLabSimpleMale.usd')

displacement_asset = viewer.import_usd_asset('../meshes/displacement.usd')

animmap = lab.AnimMapper(character, keep_translation=False, root_motion=True, match_effectors=True, local_offsets={'Hips':[0, 2, 0]})
animations = []
animations.append(lab.import_bvh(f'../../Assets/Animation/ExternalData/lafan1/bvh/walk1_subject1.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk1_subject2.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk1_subject5.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk2_subject1.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk2_subject3.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk2_subject4.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk3_subject1.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk3_subject2.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk3_subject3.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk3_subject4.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk3_subject5.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/walk4_subject1.bvh', anim_mapper=animmap))
#
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/run1_subject2.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/run1_subject5.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/run2_subject1.bvh', anim_mapper=animmap))
# animations.append(lab.import_bvh(f'../../resources/lafan1/bvh/run2_subject4.bvh', anim_mapper=animmap))

bone_count = character.bone_count()
bones = animations[0].bones
parents = animations[0].parents

contacts = []
for anim in animations:
    _, p = lab.utils.quat_fk(anim.quats, anim.pos, parents)
    contacts.append(lab.utils.extract_feet_contacts(p, bones.index('LeftToe'), bones.index('RightToe'), 0.1))

default_skeleton_p = (animations[0].pos[0, ...]).copy()
default_skeleton_p[0] = 0

POSESHAPE = (bone_count + 2, 4)


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

    root = (1.0 - t) * a.root + (t) * b.root
    hips = (1.0 - t) * a.hips + (t) * b.hips
    quats = lab.utils.normalize(lab.utils.quat_slerp(a.quats, b.quats, t))

    return pose_pack(root, hips, quats)


def pose_blend(states, weights):
    states = pose_unpack(states)

    root = np.sum(states.root * weights[:, np.newaxis], axis=0)
    hips = np.sum(states.hips * weights[:, np.newaxis], axis=0)
    quats = lab.utils.normalize(np.sum(states.quats * weights[:, np.newaxis, np.newaxis], axis=0))

    return pose_pack(root, hips, quats)


def pose_to_qp(a):
    a = pose_unpack(a)
    p = default_skeleton_p.copy()
    p[0, :] = a.root
    p[1, :] = a.hips
    return a.quats.copy(), p


states_x = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
states_v = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
states_y = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
states_c = np.zeros([50000, 2], dtype=np.bool)
states_count = 0


def add_states_ex(quats, pos, lcontact, rcontact):
    global states_count

    for i in range(quats.shape[0] - 2):
        a = pose_pack(pos[i, 0, :].copy(), pos[i, 1, :].copy(), quats[i, ...].copy())
        b = pose_pack(pos[i + 1, 0, :].copy(), pos[i + 1, 1, :].copy(), quats[i + 1, ...].copy())
        c = pose_pack(pos[i + 2, 0, :].copy(), pos[i + 2, 1, :].copy(), quats[i + 2, ...].copy())

        y = pose_subtract(c, b)
        v = pose_subtract(b, a)
        a = pose_unpack(a)
        a.root[:] = 0
        a.quats[0, :] = [1, 0, 0, 0]
        a = pose_pack(a.root, a.hips, a.quats)

        states_x[states_count, :, :] = a
        states_v[states_count, :, :] = v
        states_y[states_count, :, :] = y

        states_c[states_count, 0] = lcontact[i]
        states_c[states_count, 1] = rcontact[i]

        states_count += 1


def add_states(anim_id, timings):
    add_states_ex(animations[anim_id].quats[timings], animations[anim_id].pos[timings], contacts[anim_id][0][timings],
                  contacts[anim_id][1][timings])


# # Walk :
# add_states(2, slice(100,2800))
# end_of_walk_ids = states_count
#
# # Jog :
# add_states(15, slice(1200,1800))
# add_states(15, slice(3450,3860))
# add_states(14, slice(180,800))
# add_states(13, slice(200,2300))

add_states(0, slice(0, 7840))

states_x = states_x[:states_count, ...]
states_v = states_v[:states_count, ...]
states_y = states_y[:states_count, ...]
states_c = states_c[:states_count, ...]

FEATURE_SHAPE = (bone_count * 2, 3)
# metric_weights = np.array([0, .3, .1, .1, .01, .01, .01,  .01, .01, .01, .01,  .01, .01, .01, .01,  .2, .5, 1, 1,  .2, .5, 1, 1 ], dtype=np.float32)
# metric_velocity_weights = np.array([1, .8, .5, .5, .5, .01, .01,  .01, .01, .01, .9,  .01, .01, .01, .9,  1.2, 1.5, 2, 0,  1.2, 1.5, 2, 0 ], dtype=np.float32)
# metric_weights = np.ones([bone_count], dtype=np.float32)
# metric_velocity_weights = np.ones([bone_count], dtype=np.float32)
metric_weights = np.array(
    [0, .3, .1, .1, .5, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .2, .5, 1, 1, .2, .5, 1, 1], dtype=np.float32)
metric_velocity_weights = np.array([1, .8, .5, .1, .1, .5, .1, 0, 0, 0, 0, 0, 0, 0, 0, 1.2, 1.5, 2, 0, 1.2, 1.5, 2, 0],
                                   dtype=np.float32)


def build_distance_metric(x, v):
    next_pose = pose_unpack(x.copy())  # pose_unpack(pose_add(x, v))
    next_pose.quats[0] = [1, 0, 0, 0]
    _, p_a = lab.utils.quat_fk(next_pose.quats, default_skeleton_p, parents)

    next_pose = pose_unpack(pose_add(pose_pack(next_pose.root, next_pose.hips, next_pose.quats), v))

    # next_pose.quats[0] = [1,0,0,0]
    _, p_b = lab.utils.quat_fk(next_pose.quats, default_skeleton_p, parents)

    return np.concatenate([p_a * metric_weights[:, np.newaxis] * .8, p_b - p_a])

metric_matrix = np.zeros([states_count, FEATURE_SHAPE[0], 3], dtype=np.float32)
for i in range(states_count):
    metric_matrix[i, ...] = build_distance_metric(states_x[i], states_v[i])

toch_knn_features = torch.from_numpy(metric_matrix).to('cuda').unsqueeze(0)


def get_nns_by_vector(vector, k):
    query = torch.from_numpy(vector).to('cuda')

    # Expand p and q to broadcast
    p_exp = toch_knn_features  # shape: (1, state_count, feature_count, 3)
    q_exp = query.unsqueeze(1)  # shape: (query_count, 1, feature_count, 3)

    diff = p_exp - q_exp  # shape: (query_count, state_count, feature_count, 3), broadcasting
    point_distances = torch.norm(diff, dim=3)  # shape: (query_count, state_count, feature_count)

    # Step 2: sum distances per set
    sum_distances = torch.sum(point_distances, dim=2)  # shape: (query_count, state_count)

    # Step 3: get k nearest sets for each query
    topk = torch.topk(sum_distances, k=k, largest=False)
    knn_indices = topk.indices.cpu().numpy()  # shape: (query_count, k)
    knn_distances = topk.values.cpu().numpy()  # shape: (query_count, k)

    return knn_indices, knn_distances

def get_k_neighbors(current_x, current_v, k=15):
    indices, distances = get_nns_by_vector(build_distance_metric(current_x, current_v)[np.newaxis, ...], k)
    indices, distances = indices[0], distances[0]
    idistances = 1.0/(distances**2 + 1e-8)
    idistances /= np.sum(idistances)
    return indices, idistances

def get_batched_k_neighbors(metric_vector, k=15):
    indices, distances = get_nns_by_vector(metric_vector, k)
    idistances = 1.0/(distances**2 + 1e-8)
    idistances /= np.sum(idistances, axis=1, keepdims=True)
    return indices, idistances

def compute_v_to_reach_state(current_x, state_id):
    next_x = pose_add(states_x[state_id], states_v[state_id])
    next_x = pose_unpack(next_x)
    x = pose_unpack(current_x)
    next_x.root += x.root
    next_x.quats[0] = lab.utils.quat_mul(x.quats[0], next_x.quats[0])
    next_x = pose_pack(next_x.root, next_x.hips, next_x.quats)
    return pose_subtract(next_x, current_x)

def compute_new_state(current_x, indices, weights, tug_ratio=.1):
    tug_indice = np.argmax(weights)
    next_v = compute_v_to_reach_state(current_x, indices[tug_indice])

    blended_v = pose_blend(states_v[indices, ...], weights)
    final_v = pose_lerp(blended_v, next_v, tug_ratio)

    blended_y = pose_blend(states_y[indices, ...], weights)
    final_y = pose_lerp(blended_y, states_y[indices[tug_indice]], tug_ratio)

    return (
        pose_add(current_x, final_v),
        final_y
    )


