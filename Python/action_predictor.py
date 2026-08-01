import numpy as np
import ipyanimlab as lab
from sympy import euler

import Skeleton
from MotionField import MotionField
from Pose import Pose
from scipy.spatial.transform import Rotation

from Skeleton import Skeleton

global pose_count

from Animation import PoseSet
from pose_set_importer import deserialize_pose_set


def load_animations():
    pose_set: PoseSet = deserialize_pose_set('../Assets/StreamingAssets/MMDatabases/MotionMatchingData', 'MotionMatchingData')

    skeleton: Skeleton = pose_set.skeleton
    bone_count = len(list(skeleton))

    n = pose_set.local_positions.shape[0]
    N = n - 2  # need triplets (i, i+1, i+2)

    pos   = pose_set.local_positions            # (n, n_joints, 3)
    quats = pose_set.local_rotations            # (n, n_joints, 4)
    lv    = pose_set.local_velocities           # (n, n_joints, 3)
    lav   = pose_set.local_angular_velocities   # (n, n_joints, 3) — Euler angles, degrees, xyz

    # Convert angular velocities (Euler angles, degrees, xyz) → unit quaternions (n, n_joints, 4)
    lav_quats = (
        Rotation.from_euler('xyz', lav.reshape(-1, 3), degrees=True)
        .as_quat()
        .reshape(n, bone_count, 4)
        .astype(np.float32)
    )

    # pose_x: current pose at frame i, expressed in root-relative frame
    #   slot 0 — root position   : zeroed  (translation-invariant)
    #   slot 1 — hip position    : kept as-is
    #   slot 2 — root bone quat  : identity (yaw-invariant)
    #   slots 3+ — remaining joint quats
    pose_x = np.zeros((N, bone_count + 2, 4), dtype=np.float32)
    pose_x[:, 1, :3] = pos[:N, 1, :] # hip
    pose_x[:, 2, :]  = [0., 0., 0., 1.]         # identity quaternion [x,y,z,w]
    pose_x[:, 3:, :] = quats[:N, 1:, :]

    # pose_v: velocity at frame i  =  Pose(i+1) − Pose(i)
    #   slot 0 — root translational velocity
    #   slot 1 — hip translational velocity
    #   slots 2+ — joint angular velocities as quaternions
    pose_v = np.zeros((N, bone_count + 2, 4), dtype=np.float32)
    pose_v[:, 0, :3] = lv[:N, 0, :]
    pose_v[:, 1, :3] = lv[:N, 1, :]
    pose_v[:, 2:, :] = lav_quats[:N, :, :]

    # pose_y: future velocity at frame i+1  =  Pose(i+2) − Pose(i+1)
    #   Same layout as pose_v, shifted one frame forward.
    pose_y = np.zeros((N, bone_count + 2, 4), dtype=np.float32)
    pose_y[:, 0, :3] = lv[1:N+1, 0, :]
    pose_y[:, 1, :3] = lv[1:N+1, 1, :]
    pose_y[:, 2:, :] = lav_quats[1:N+1, :, :]

    # pose_contacts: foot contact flags aligned to frame i
    pose_contacts = pose_set.foot_contacts[:N, :].copy()

    return skeleton, pose_x, pose_v, pose_y, pose_contacts


import zmq, json


def create_pose_json_reply(skeleton: Skeleton,
                           current_x: np.ndarray,
                           current_v: np.ndarray,
                           pose_contacts: np.ndarray):
    p = Pose.from_array(current_x)
    local_positions = skeleton.get_local_positions(p)[0]
    assert local_positions.shape == (23, 3), f"Expected shape {(23, 3)}, got {local_positions.shape}"
    euler_angles = np.array(Rotation.from_quat(Pose.from_array(current_v).quats[0]).as_euler('xyz'))

    return {
        "JointLocalPositions": [{"x": pos[0], "y": pos[1], "z": pos[2]} for pos in local_positions.tolist()],
        "JointLocalRotations": [{"value": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}} for quat in
                                p.quats[0].tolist()],
        "JointLocalVelocities": [{"x": 0.0, "y": 0.0, "z": 0.0} for _ in range(23)],
        "JointLocalAngularVelocities": [{"x": ang_vel[0], "y": ang_vel[1], "z": ang_vel[2]} for ang_vel in
                                        euler_angles.tolist()],
        "LeftFootContact": bool(pose_contacts[0]),
        "RightFootContact": bool(pose_contacts[1])
    }


def main():
    skeleton, pose_x, pose_v, pose_y, pose_contacts = load_animations()
    motion_field = MotionField(pose_x, pose_v, pose_y, skeleton)

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5557")
    print("Python ZeroMQ server listening on port 5555...")

    last_index = 600
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

        current_x[...], current_v[...] = motion_field.greedy_action(
            desired_dir, current_x, current_v, delta_time, k_neighbors=15)
        reply = create_pose_json_reply(skeleton, current_x, current_v, current_contacts)
        socket.send_string(json.dumps(reply))



if __name__ == "__main__":
    main()
