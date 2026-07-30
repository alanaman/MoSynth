import pickle
from dataclasses import dataclass, field
from random import randrange
import numpy as np
from ipywidgets import widgets, interact
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as Rot

import torch
import torch.nn as nn

import ipyanimlab as lab

from Pose import Pose

viewer = lab.Viewer(move_speed=5, width=1280, height=720)

# load the character
character = viewer.import_usd_asset('AnimLabSimpleMale.usd')

animmap = lab.AnimMapper(character, keep_translation=False, root_motion=True, match_effectors=True,
                         local_offsets={'Hips': [0, 2, 0]})
displacement_asset = viewer.import_usd_asset('meshes/displacement.usd')
# %%
animations = []
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk1_subject1.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk1_subject2.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk1_subject5.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk2_subject1.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk2_subject3.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk2_subject4.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk3_subject1.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk3_subject2.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk3_subject3.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk3_subject4.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk3_subject5.bvh', anim_mapper=animmap))
animations.append(
    lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/walk4_subject1.bvh', anim_mapper=animmap))
animations.append(lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run1_subject2.bvh', anim_mapper=animmap))
animations.append(lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run1_subject5.bvh', anim_mapper=animmap))
animations.append(lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run2_subject1.bvh', anim_mapper=animmap))
animations.append(lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run2_subject4.bvh', anim_mapper=animmap))

bone_count = character.bone_count()
bones = animations[0].bones
parents = animations[0].parents
default_skeleton_p = (animations[0].pos[0, ...]).copy()
default_skeleton_p[0] = 0

contacts = []
for anim in animations:
    _, p = lab.utils.quat_fk(anim.quats, anim.pos, parents)
    contacts.append(lab.utils.extract_feet_contacts(p, bones.index('LeftToe'), bones.index('RightToe'), 0.1))


states_x = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
states_v = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
states_y = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
states_c = np.zeros([50000, 2], dtype=np.bool)
states_count = 0


def add_states_ex(quats, pos, l_contact, r_contact):
    global states_count

    for i in range(quats.shape[0] - 2):
        a = Pose(
            pos[i, 0, :].copy(),
            pos[i, 1, :].copy(),
            Rot.from_quat(quats[i, ...], scalar_first=True))
        b = Pose(
            pos[i + 1, 0, :].copy(),
            pos[i + 1, 1, :].copy(),
            Rot.from_quat(quats[i + 1, ...], scalar_first=True))
        c = Pose(
            pos[i + 2, 0, :].copy(),
            pos[i + 2, 1, :].copy(),
            Rot.from_quat(quats[i + 2, ...], scalar_first=True))

        y = c - b
        v = b - a
        a.rootPos[:] = 0
        a.quats[0, :] = Rot.identity()

        states_x[states_count, :, :] = a.pack()
        states_v[states_count, :, :] = v.pack()
        states_y[states_count, :, :] = y.pack()

        states_c[states_count, 0] = l_contact[i]
        states_c[states_count, 1] = r_contact[i]

        states_count += 1


def add_states(anim_id, timings):
    add_states_ex(animations[anim_id].quats[timings], animations[anim_id].pos[timings], contacts[anim_id][0][timings],
                  contacts[anim_id][1][timings])


# Walk :
add_states(2, slice(100, 2800))
end_of_walk_ids = states_count

# Jog :
add_states(15, slice(1200, 1800))
add_states(15, slice(3450, 3860))
add_states(14, slice(180, 800))
add_states(13, slice(200, 2300))

states_x = states_x[:states_count, ...]
states_v = states_v[:states_count, ...]
states_y = states_y[:states_count, ...]
states_c = states_c[:states_count, ...]

FEATURE_SHAPE = (bone_count * 2, 3)
metric_weights = np.array(
    [
        0, .3, .1, .1, .5, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .2, .5, 1, 1, .2, .5, 1, 1],
    dtype=np.float32)
metric_velocity_weights = np.array(
    [1, .8, .5, .1, .1, .5, .1, 0, 0, 0, 0, 0, 0, 0, 0, 1.2, 1.5, 2, 0, 1.2, 1.5, 2, 0],
    dtype=np.float32)


def build_motion_state(x, v):
    next_pose = Pose.from_array(x)  # pose_unpack(pose_add(x, v))
    next_pose.quats[0] = Rot.identity()

    _, p_a = lab.utils.quat_fk(next_pose.quats, default_skeleton_p, parents)

    next_pose = Pose(next_pose.rootPos, next_pose.hipPos, next_pose.quats) + v

    # next_pose.quats[0] = [1,0,0,0]
    _, p_b = lab.utils.quat_fk(next_pose.quats, default_skeleton_p, parents)

    return np.concatenate([p_a * metric_weights[:, np.newaxis] * .8, p_b - p_a])


motion_states = np.zeros([states_count, FEATURE_SHAPE[0], 3], dtype=np.float32)
for i in range(states_count):
    motion_states[i, ...] = build_motion_state(states_x[i], states_v[i])


##### test till here

