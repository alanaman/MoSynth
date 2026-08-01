import os
import struct
import numpy as np

from Animation import PoseSet
from Skeleton import Joint, Skeleton


# Assuming Skeleton and PoseSet are imported/defined here
# from Skeleton import Skeleton
# from PoseSet import PoseSet

def _read_csharp_string(f) -> str:
    """
    Reads a string written by C# BinaryWriter.
    C# BinaryWriter prefixes strings with a LEB128 (7-bit encoded) length.
    """
    count = 0
    shift = 0
    while True:
        b = f.read(1)
        if not b:
            return ""
        b = b[0]
        count |= (b & 0x7F) << shift
        shift += 7
        if not (b & 0x80):
            break

    if count == 0:
        return ""
    return f.read(count).decode('utf-8')


def read_skeleton(filepath: str) -> Skeleton:
    """
    Reads the .mmskeleton file.
    Note: Adjust the instantiation to match your actual Python Skeleton class definition.
    """
    # Initialize your Skeleton object here
    skeleton_data = []

    with open(filepath, 'rb') as f:
        n_joints = struct.unpack('<I', f.read(4))[0]

        for _ in range(n_joints):
            name = _read_csharp_string(f)
            joint_index = struct.unpack('<I', f.read(4))[0]
            parent_index = struct.unpack('<i', f.read(4))[0]
            local_offset = struct.unpack('<3f', f.read(12))  # x, y, z
            joint_type = struct.unpack('<I', f.read(4))[0]

            skeleton_data.append({
                'name': name,
                'index': joint_index,
                'parent_index': parent_index,
                'local_offset': local_offset,
                'type': joint_type
            })
    joints = [Joint(datum['name'], datum['local_offset']) for datum in skeleton_data]
    for joint_idx, joint in enumerate(joints):
        datum = skeleton_data[joint_idx]
        parent_idx = int(datum['parent_index'])
        if parent_idx == -1: continue
        parent_joint = joints[parent_idx]
        parent_joint.add_child(joint)

    assert joints[0].parent() is None, "root bone should be the first bone"

    skeleton = Skeleton("char", joints[0])
    # return Skeleton(skeleton_data)
    return skeleton  # Placeholder: return data directly if Skeleton constructor differs


def deserialize_pose_set(path: str, file_name: str) -> PoseSet:
    """
    Reads .mmskeleton and .mmpose files from the given path and constructs a PoseSet object.
    """
    skeleton_path = os.path.join(path, f"{file_name}.mmskeleton")
    pose_path = os.path.join(path, f"{file_name}.mmpose")

    if not os.path.exists(skeleton_path) or not os.path.exists(pose_path):
        raise FileNotFoundError(f"Could not find .mmskeleton or .mmpose files at {path}")

    # 1. Deserialize Skeleton
    skeleton = read_skeleton(skeleton_path)

    # 2. Deserialize Pose Data
    with open(pose_path, 'rb') as f:
        # Read Clips
        n_clips = struct.unpack('<I', f.read(4))[0]
        clips = []
        frame_time = 0.0
        for _ in range(n_clips):
            start = struct.unpack('<I', f.read(4))[0]
            end = struct.unpack('<I', f.read(4))[0]
            f_time = struct.unpack('<f', f.read(4))[0]
            clips.append({'start': start, 'end': end})
            frame_time = f_time  # Overwritten per clip, usually constant across a PoseSet

        # Read Header Counts
        n_poses, n_joints, n_tags = struct.unpack('<I I I', f.read(12))

        # Pre-allocate numpy arrays for fast assignment
        pos = np.zeros((n_poses, n_joints, 3), dtype=np.float32)
        quats = np.zeros((n_poses, n_joints, 4), dtype=np.float32)
        vel = np.zeros((n_poses, n_joints, 3), dtype=np.float32)
        ang_vel = np.zeros((n_poses, n_joints, 3), dtype=np.float32)

        foot_contacts = np.zeros((n_poses, 2), dtype=bool)

        # Read Poses
        for i in range(n_poses):
            # JointLocalPositions (float3 = 12 bytes per joint)
            pos[i] = np.frombuffer(f.read(n_joints * 12), dtype=np.float32).reshape(n_joints, 3)

            # JointLocalRotations (quaternion = 16 bytes per joint)
            quats[i] = np.frombuffer(f.read(n_joints * 16), dtype=np.float32).reshape(n_joints, 4)

            # JointLocalVelocities
            vel[i] = np.frombuffer(f.read(n_joints * 12), dtype=np.float32).reshape(n_joints, 3)

            # JointLocalAngularVelocities
            ang_vel[i] = np.frombuffer(f.read(n_joints * 12), dtype=np.float32).reshape(n_joints, 3)

            # Foot Contacts (uint = 4 bytes each)
            left_contact = struct.unpack('<I', f.read(4))[0] == 1
            right_contact = struct.unpack('<I', f.read(4))[0] == 1
            foot_contacts[i] = [left_contact, right_contact]

        # Read Tags
        tags = []
        for _ in range(n_tags):
            tag_name = _read_csharp_string(f)
            n_ranges = struct.unpack('<I', f.read(4))[0]

            ranges = []
            for _ in range(n_ranges):
                start_range = struct.unpack('<I', f.read(4))[0]
                end_range = struct.unpack('<I', f.read(4))[0]
                ranges.append((start_range, end_range))

            tags.append({
                'name': tag_name,
                'ranges': ranges
            })

    # 3. Instantiate the Python PoseSet
    pose_set = PoseSet(
        skeleton=skeleton,
        frame_time=frame_time,
        local_pos=pos,
        local_quats=quats,
        foot_contacts=foot_contacts,
        local_vel=vel,
        local_angular_vel=ang_vel,
        clips=clips,
    )

    # Optionally attach data not directly in the __init__ definition to the python object
    pose_set.tags = tags

    return pose_set

# Example Usage:
# my_pose_set = deserialize_pose_set("path/to/directory", "RunAnimation")
