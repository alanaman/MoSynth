import json

import numpy as np

from MotionField import MotionField
from Pose import Pose, PoseDelta
from Skeleton import Skeleton

global pose_count

from Animation import PoseSet
from pose_set_importer import deserialize_pose_set

from debugging.python_net import connect_debugger

connect_debugger()


def load_animations(data_dir='../Assets/StreamingAssets/MMDatabases/MotionMatchingData'):
    pose_set: PoseSet = deserialize_pose_set(data_dir, 'MotionMatchingData')

    skeleton: Skeleton = pose_set.skeleton
    bone_count = len(list(skeleton))

    n_src_poses = pose_set.local_positions.shape[0]
    n_target_poses = n_src_poses - 1  # need (i, i+1) i has x, v and i+1 has y

    pos = pose_set.local_positions  # (n_poses, n_joints, 3)
    quats = pose_set.local_rotations  # (n_poses, n_joints, 4)
    lv = pose_set.local_velocities  # (n_poses, n_joints, 3) — m/s
    lav = pose_set.local_angular_velocities  # (n_poses, n_joints, 3) — rotation vectors, rad/s, xyz

    # NOTE: lv/lav are per-SECOND rates (PoseExtractor divides by FrameTime), and
    # they are stored here as-is. Integrating them requires an explicit time step:
    #   pose_x[i].add(PoseDelta.from_array(pose_v[i]).scaled(frame_time)) == pose_x[i+1]
    # Angular rates stay as rotation vectors; see PoseDelta for why converting
    # them to quaternions here would alias every joint faster than pi rad/s.

    # pose_x: current pose at frame i, expressed in root-relative frame
    #   slot 0 — root position: zeroed (translation-invariant)
    #   slot 1 — hip position: kept as-is
    #   slot 2 — root bone quat: identity (yaw-invariant)
    #   slots 3+ — remaining joint quats
    pose_x = np.zeros((n_target_poses, bone_count + 2, 4), dtype=np.float32)
    pose_x[:, 1, :3] = pos[:n_target_poses, 1, :]  # hip
    pose_x[:, 2, :] = [0., 0., 0., 1.]  # identity quaternion [x,y,z,w]
    pose_x[:, 3:, :] = quats[:n_target_poses, 1:, :]

    # pose_v: velocity at frame i  =  (Pose(i+1) − Pose(i)) / frame_time
    #   slot 0 — root translational rate (root-local space)
    #   slot 1 — hip translational rate
    #   slots 2+ — joint angular rates as rotation vectors (trailing component unused)
    pose_v = np.zeros((n_target_poses, bone_count + 2, 4), dtype=np.float32)
    pose_v[:, 0, :3] = lv[:n_target_poses, 0, :]
    pose_v[:, 1, :3] = lv[:n_target_poses, 1, :]
    pose_v[:, 2:, :3] = lav[:n_target_poses, :, :]

    # pose_y: future velocity at frame i+1  =  (Pose(i+2) − Pose(i+1)) / frame_time
    #   Same layout as pose_v, shifted one frame forward.
    pose_y = np.zeros((n_target_poses, bone_count + 2, 4), dtype=np.float32)
    pose_y[:, 0, :3] = lv[1:n_target_poses + 1, 0, :]
    pose_y[:, 1, :3] = lv[1:n_target_poses + 1, 1, :]
    pose_y[:, 2:, :3] = lav[1:n_target_poses + 1, :, :]

    # pose_contacts: foot contact flags aligned to frame i
    pose_contacts = pose_set.foot_contacts[:n_target_poses, :].copy()

    return skeleton, pose_x, pose_v, pose_y, pose_contacts, pose_set.frameTime, pose_set


import zmq


def create_pose_json_reply(skeleton: Skeleton,
                           current_x: np.ndarray,
                           current_v: np.ndarray,
                           pose_contacts: np.ndarray):
    p_x = Pose.from_array(current_x)
    p_v = PoseDelta.from_array(current_v)

    pos = skeleton.get_local_positions(p_x)[0]
    assert pos.shape == (23, 3), f"Expected shape {(23, 3)}, got {pos.shape}"
    lv = np.zeros_like(pos)
    lv[0] = p_v.rootVel
    quats = p_x.quats[0]
    lav = p_v.rotvecs[0]
    return {
        "JointLocalPositions": [{"x": p[0], "y": p[1], "z": p[2]} for p in pos.tolist()],
        "JointLocalRotations": [{"value": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]}} for q in quats.tolist()],
        "JointLocalVelocities": [{"x": v[0], "y": v[1], "z": v[2]} for v in lv.tolist()],
        "JointLocalAngularVelocities": [{"x": ang_vel[0], "y": ang_vel[1], "z": ang_vel[2]} for ang_vel in
                                        lav.tolist()],
        "LeftFootContact": bool(pose_contacts[0]),
        "RightFootContact": bool(pose_contacts[1])
    }

def get_pose_arrays(skeleton: Skeleton,
                    current_x: np.ndarray,
                    current_v: np.ndarray,
                    pose_contacts: np.ndarray):
    p_x = Pose.from_array(current_x)
    p_v = PoseDelta.from_array(current_v)

    pos = skeleton.get_local_positions(p_x)[0]
    lv = np.zeros_like(pos)
    lv[0] = p_v.rootVel
    quats = p_x.quats[0]
    lav = p_v.rotvecs[0]

    # Ensure flat float arrays for PythonNet auto-marshalling
    pos = np.ascontiguousarray(pos, dtype=np.float32).flatten().tolist()
    quats = np.ascontiguousarray(quats, dtype=np.float32).flatten().tolist()
    lv = np.ascontiguousarray(lv, dtype=np.float32).flatten().tolist()
    lav = np.ascontiguousarray(lav, dtype=np.float32).flatten().tolist()

    return pos, quats, lv, lav, bool(pose_contacts[0]), bool(pose_contacts[1])


def main():
    skeleton, pose_x, pose_v, pose_y, pose_contacts, frame_time, pose_set = load_animations()
    motion_field = MotionField(pose_x, pose_v, pose_y, skeleton, frame_time)

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    print("Python ZeroMQ server listening on port 5555...")

    last_index = 700
    current_x = pose_x[last_index, ...].copy()
    current_v = pose_v[last_index, ...].copy()
    current_contacts = pose_contacts[last_index, ...]

    while True:

        raw_msg = socket.recv()

        # decode desired direction from json sent from Unity
        try:
            msg = raw_msg.decode('utf-8')
            data = json.loads(msg)
            desired_dir = np.array(data["desired_dir"])
            delta_time = float(data["delta_time"])
        except Exception as e:
            print(f"Error decoding message: {e}")
            socket.send_string(json.dumps({"error": "Invalid desired direction format"}))
            continue

        # desired_dir = np.array([0.0, 0.0])
        current_x[...], current_v[...] = motion_field.get_next_pose(current_x, current_v, delta_time)
        # current_x[...], current_v[...] = motion_field.get_next_pose_blended(current_x, current_v, delta_time, k_neighbors=15)
#         current_x[...], current_v[...] = motion_field.greedy_action(desired_dir, current_x, current_v, delta_time, k_neighbors=15)
        reply = create_pose_json_reply(skeleton, current_x, current_v, current_contacts)
        socket.send_string(json.dumps(reply))


if __name__ == "__main__":
    main()

def get_multiplier():
    return 10
