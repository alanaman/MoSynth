import numpy as np
import ipyanimlab as lab

from MotionField import MotionField
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

from Skeleton import Skeleton, Joint

joints = [Joint(bone_name) for bone_name in bones]
for joint_idx, joint in enumerate(joints):
    parent_idx = int(parents[joint_idx])
    if parent_idx == -1: continue
    parent_joint = joints[parent_idx]
    parent_joint.add_child(joint)

assert joints[0].parent() is None, "root bone should be the first bone"

skeleton = Skeleton("char", joints[0])

contacts = []
for anim in animations:
    _, p = lab.utils.quat_fk(anim.quats, anim.pos, parents)
    contacts.append(lab.utils.extract_feet_contacts(p, bones.index('LeftToe'), bones.index('RightToe'), 0.1))

pose_x = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
pose_v = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
pose_y = np.zeros([50000, bone_count + 2, 4], dtype=np.float32)
pose_c = np.zeros([50000, 2], dtype=np.bool)
pose_count = 0


def add_poses_ex(quats, pos, l_contact, r_contact):
    global pose_count

    for i in range(quats.shape[0] - 2):
        a = Pose(
            pos[i, 0, :].copy(),
            pos[i, 1, :].copy(),
            np.array(
                Rot.from_quat(quats[i, ...], scalar_first=True).as_quat()
            ))
        b = Pose(
            pos[i + 1, 0, :].copy(),
            pos[i + 1, 1, :].copy(),
            np.array(
                Rot.from_quat(quats[i + 1, ...], scalar_first=True).as_quat()
            ))
        c = Pose(
            pos[i + 2, 0, :].copy(),
            pos[i + 2, 1, :].copy(),
            np.array(
                Rot.from_quat(quats[i + 2, ...], scalar_first=True).as_quat()
            ))

        y = c - b
        v = b - a
        a.rootPos[:] = 0
        a.quats[..., 0, :] = np.array(Rot.identity().as_quat())

        pose_x[pose_count, :, :] = a.pack()
        pose_v[pose_count, :, :] = v.pack()
        pose_y[pose_count, :, :] = y.pack()

        pose_c[pose_count, 0] = l_contact[i]
        pose_c[pose_count, 1] = r_contact[i]

        pose_count += 1


def add_poses(anim_id, timings):
    add_poses_ex(animations[anim_id].quats[timings], animations[anim_id].pos[timings], contacts[anim_id][0][timings],
                 contacts[anim_id][1][timings])


# Walk :
add_poses(2, slice(100, 2800))
end_of_walk_ids = pose_count

# Jog :
add_poses(15, slice(1200, 1800))
add_poses(15, slice(3450, 3860))
add_poses(14, slice(180, 800))
add_poses(13, slice(200, 2300))

pose_x = pose_x[:pose_count, ...]
pose_v = pose_v[:pose_count, ...]
pose_y = pose_y[:pose_count, ...]
pose_c = pose_c[:pose_count, ...]

FEATURE_SHAPE = (bone_count * 2, 3)
metric_weights = np.array(
    [
        0, .3, .1, .1, .5, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .2, .5, 1, 1, .2, .5, 1, 1],
    dtype=np.float32)
metric_velocity_weights = np.array(
    [1, .8, .5, .1, .1, .5, .1, 0, 0, 0, 0, 0, 0, 0, 0, 1.2, 1.5, 2, 0, 1.2, 1.5, 2, 0],
    dtype=np.float32)


def build_motion_states(x, v):
    current_pose = Pose.from_array(x)

    p_a, _ = skeleton.fk_root_space(current_pose)
    # _, p_a = lab.utils.quat_fk(next_pose.quats, default_skeleton_p, parents)

    next_pose = current_pose + Pose.from_array(v)

    # _, p_b = lab.utils.quat_fk(next_pose.quats, default_skeleton_p, parents)
    p_b, _ = skeleton.fk_root_space(next_pose)

    return np.concatenate([p_a * metric_weights[:, np.newaxis] * .8, p_b - p_a], axis=-2) # (batch, bone_count*2, 3)

motion_states = build_motion_states(pose_x, pose_v)

motion_field = MotionField(motion_states)

from scipy.spatial.transform import Rotation as Rot


desired_dir = client.getdir()
current_x = client.getx()
current_v = client.getvel()

current_state = build_motion_states(current_x, current_v)

K_NEIGHBORS = 15

indices, distances = motion_field.get_knn(current_state, k=K_NEIGHBORS)
weights = MotionField.calculate_similarity_weights(distances)

rewards = np.zeros(K_NEIGHBORS)

for n_idx in range(K_NEIGHBORS):
    w = weights.copy()
    w[n_idx] = 1.0
    w /= np.sum(w)
    nx, nv = motion_field.compute_new_state(current_x, indices, w)

    Pose.from_array(nx).quats[0]

    reward = action_reward(desired_direction, current_x, nx)

    # store the rewards
    rewards[n_idx] = reward

motion_field.compute_new_state(current_state)

