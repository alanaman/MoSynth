import numpy as np
import ipyanimlab as lab
from sympy import euler

import Skeleton
# from MotionField import MotionField
from Pose import Pose
from scipy.spatial.transform import Rotation

from Skeleton import Skeleton

global pose_count


def load_animations():
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
    animations.append(
        lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run1_subject2.bvh', anim_mapper=animmap))
    animations.append(
        lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run1_subject5.bvh', anim_mapper=animmap))
    animations.append(
        lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run2_subject1.bvh', anim_mapper=animmap))
    animations.append(
        lab.import_bvh(f'../Assets/Animation/ExternalData/lafan1/bvh/run2_subject4.bvh', anim_mapper=animmap))

    bone_count = character.bone_count()
    bones = animations[0].bones
    parents = animations[0].parents
    default_skeleton_p = (animations[0].pos[0, ...]).copy()
    default_skeleton_p[0] = 0

    from Skeleton import Skeleton, Joint

    joints = [Joint(bone_name, default_skeleton_p[i]) for i, bone_name in enumerate(bones)]
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
    pose_contacts = np.zeros([50000, 2], dtype=np.bool)

    global pose_count
    pose_count = 0

    def add_poses_ex(quats, pos, l_contact, r_contact):

        global pose_count
        for i in range(quats.shape[0] - 2):
            a = Pose(
                pos[i, 0, :].copy(),
                pos[i, 1, :].copy(),
                np.array(
                    Rotation.from_quat(quats[i, ...], scalar_first=True).as_quat()
                ))
            b = Pose(
                pos[i + 1, 0, :].copy(),
                pos[i + 1, 1, :].copy(),
                np.array(
                    Rotation.from_quat(quats[i + 1, ...], scalar_first=True).as_quat()
                ))
            c = Pose(
                pos[i + 2, 0, :].copy(),
                pos[i + 2, 1, :].copy(),
                np.array(
                    Rotation.from_quat(quats[i + 2, ...], scalar_first=True).as_quat()
                ))

            y = c - b
            v = b - a
            a.rootPos[:] = 0
            a.quats[..., 0, :] = np.array(Rotation.identity().as_quat())

            pose_x[pose_count, :, :] = a.pack()
            pose_v[pose_count, :, :] = v.pack()
            pose_y[pose_count, :, :] = y.pack()

            pose_contacts[pose_count, 0] = l_contact[i]
            pose_contacts[pose_count, 1] = r_contact[i]

            pose_count += 1

    def add_poses(anim_id, timings):
        add_poses_ex(animations[anim_id].quats[timings], animations[anim_id].pos[timings],
                     contacts[anim_id][0][timings],
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
    pose_contacts = pose_contacts[:pose_count, ...]

    return skeleton, pose_x, pose_v, pose_y, pose_contacts


import zmq, json


def create_pose_json_reply(skeleton: Skeleton,
                           current_x: np.ndarray,
                           current_v: np.ndarray,
                           pose_contacts: np.ndarray):
    p = Pose.from_array(current_x)
    local_positions = skeleton.get_local_positions(p)
    assert local_positions.shape == (23, 3), f"Expected shape {(23, 3)}, got {local_positions.shape}"
    euler_angles = np.array(Rotation.from_quat(Pose.from_array(current_v).quats).as_euler('xyz'))

    return {
        "JointLocalPositions": [{"x": float(pos[0]), "y": float(pos[1]), "z": float(-pos[2])} for pos in local_positions.tolist()],
        "JointLocalRotations": [{"value": {"x": float(-quat[0]), "y": float(-quat[1]), "z": float(quat[2]), "w": float(quat[3])}} for quat in p.quats.tolist()],
        "JointLocalVelocities": [{"x": 0.0, "y": 0.0, "z": 0.0} for _ in range(23)],
        "JointLocalAngularVelocities": [{"x": float(-ang_vel[0]), "y": float(-ang_vel[1]), "z": float(ang_vel[2])} for ang_vel in euler_angles.tolist()],
        "LeftFootContact": bool(pose_contacts[0]),
        "RightFootContact": bool(pose_contacts[1])
    }


def main():
    skeleton, pose_x, pose_v, pose_y, pose_contacts = load_animations()
    # motion_field = MotionField(pose_x, pose_v, pose_y, skeleton)

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5557")
    print("Python ZeroMQ server listening on port 5555...")

    last_index = 400
    current_x = pose_x[last_index, ...].copy()
    current_v = pose_v[last_index, ...].copy()
    current_contacts = pose_contacts[last_index, ...]

    while True:
        raw_msg = socket.recv()

        # # decode desired direction from json sent from Unity
        # try:
        #     msg = raw_msg.decode('utf-8')
        #     data = json.loads(msg)
        #     desired_dir = np.array(data["desired_dir"])
        # except Exception as e:
        #     print(f"Error decoding message: {e}")
        #     socket.send_string(json.dumps({"error": "Invalid desired direction format"}))
        #     continue

        reply = create_pose_json_reply(skeleton, current_x, current_v, current_contacts)
        socket.send_string(json.dumps(reply))

        # current_x[...], current_v[...] = motion_field.greedy_action(
        #     desired_dir, current_x, current_v, k_neighbors=15)


if __name__ == "__main__":
    main()
