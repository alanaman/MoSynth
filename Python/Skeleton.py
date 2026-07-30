from __future__ import annotations
from typing import Union, Iterator, Tuple

from scipy.spatial.transform import Rotation

from Pose import Pose
import numpy as np


class Skeleton:
    def __init__(self, name, root_joint: Joint):
        self.name = name
        self.root_joint = root_joint

    def __iter__(self) -> Iterator[Joint]:
        """Iterates joints in depth-first order starting from the root_joint."""
        return iter(self.root_joint)

    def fk_root_space(self, pose: Pose) -> tuple[np.ndarray, Rotation]:
        """
        Performs forward kinematics while ignoring the root.
        Returns a tuple of:
            - Root space positions: np.ndarray of shape (..., num_joints, 3)
            - World space rotations: scipy.spatial.transform.Rotation of shape (..., num_joints)
        """

        pose.quats

        joints = list(self)
        num_joints = len(joints)
        joint_to_idx = {j: i for i, j in enumerate(joints)}

        batch_shape = pose.quats.shape[:-1]

        global_positions = [
            np.zeros(batch_shape + (num_joints, 3), dtype=np.float32)]
        global_rotations = [Rotation.from_quat(
            np.zeros(batch_shape + (num_joints, 4), dtype=np.float32))]

        for i, joint in enumerate(joints):
            if joint.parent is None: continue

            local_rot = pose.quats[..., i]
            local_pos = joint.default_local_position
            if i == 1:  # hips
                local_pos = pose.hipPos

            parent = joint.parent
            parent_idx = joint_to_idx[parent]
            parent_rot = global_rotations[parent_idx]
            parent_pos = global_positions[..., parent_idx, :]

            global_rotations.append(parent_rot * local_rot)
            global_positions.append(parent_pos + parent_rot.apply(local_pos))

        # Combine rotations into a single Rotation object of shape (..., num_joints)
        all_quats = np.stack([r.as_quat() for r in global_rotations], axis=-2)
        return np.stack(global_positions), Rotation.from_quat(all_quats)


class Joint:
    def __init__(self, name):
        self.name = name
        self.parent = None
        self.children: list[Joint] = []
        self.default_local_position = np.zeros(3, dtype=np.float32)
        self.default_local_rotation = Rotation.identity()

    def add_child(self, child: Joint):
        assert child.parent is None, "Child joint already has a parent."
        self.children.append(child)
        child.parent = self

    def dfs(self) -> Iterator[Joint]:
        """Generator that yields this joint and all descendants in depth-first order."""
        yield self
        for child in self.children:
            yield from child.dfs()

    def __iter__(self) -> Iterator[Joint]:
        """Allow `iter(joint)` to iterate DFS from this joint."""
        return self.dfs()
