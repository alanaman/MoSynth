from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
from typing import List, Tuple, Union


class Pose:
    """
    Represents skeletal pose data with root translation, hip translation,
    and joint rotation quaternions.
    """

    rootPos: np.ndarray
    hipPos: np.ndarray
    quats: Rotation

    def __init__(self, root: np.ndarray, hips: np.ndarray, quats: Rotation):
        """
        :param root: Vector3 root position (..., 3)
        :param hips: Vector3 hips position (..., 3)
        :param quats: Joint rotations SciPy Rotation object (..., num_bones, ).
        """
        assert root.shape[-1] == 3, "Root position must be a 3D vector."
        assert hips.shape[-1] == 3, "Hips position must be a 3D vector."
        assert root.shape[:-1] == hips.shape[:-1] and root.shape[:-1] == quats.shape[:-1], "Batch sizes must match."

        self.rootPos = np.asarray(root, dtype=np.float32)
        self.hipPos = np.asarray(hips, dtype=np.float32)
        self.quats = quats

    @classmethod
    def from_array(cls, pose: np.ndarray) -> Pose:
        """Unpacks a flat pose tensor into a PoseData instance."""
        root = pose[..., 0, :3].copy()
        hips = pose[..., 1, :3].copy()
        quats = pose[..., 2:, :].copy()
        return Pose(root, hips, Rotation.from_quat(quats))

    def pack(self) -> np.ndarray:
        """Packs the PoseData back into a single flat array representation."""
        # Get raw quaternion array to determine shape
        raw_quats : np.ndarray = self.quats.as_quat()
        num_bones = raw_quats.shape[-2]
        batch_shape = raw_quats.shape[:-2]

        result = np.zeros((*batch_shape, num_bones + 2, 4), dtype=np.float32)
        result[..., 0, :3] = self.rootPos
        result[..., 1, :3] = self.hipPos
        result[..., 2:, :] = raw_quats
        return result

    def __add__(self, other: Pose) -> Pose:
        """
        Adds a velocity/delta pose to this pose.
        (Calculates relative transformation for root and composes rotations).
        """
        # Extract root rotations (assuming index 0 is the root bone)
        # Using slicing [0:1] to maintain shape or indexing [0] based on usage
        # We assume the user wants the first rotation in the sequence.

        r_x0: Rotation = self.quats[0]
        r_v0: Rotation = other.quats[0]

        # Multiply rotation and rotate offset vector
        new_root = self.rootPos + r_x0.apply(other.rootPos)

        # Hips vector addition
        new_hips = self.hipPos + other.hipPos

        # Joint rotations composition
        new_quats = self.quats * other.quats

        return Pose(new_root, new_hips, new_quats)

    def __sub__(self, other: Pose) -> Pose:
        """
        Subtracts a base pose 'other' from 'self' to get the delta pose.
        """
        # Inverse root rotation of 'other' (b)
        r_b0_inv: Rotation = other.quats[0].inv()

        # Relative root position & rotation
        diff_root : np.ndarray = r_b0_inv.apply(self.rootPos - other.rootPos)

        # Hips vector difference
        diff_hips = self.hipPos - other.hipPos

        # Relative joint rotations: inverse(b) * a
        diff_quats = other.quats.inv() * self.quats

        # Extract underlying array for shortest path sign alignment
        raw_diff_quats: np.ndarray = diff_quats.as_quat()

        # Because we use [x, y, z, w], the scalar part 'w' is now at index -1
        flip = raw_diff_quats[..., -1] < 0
        raw_diff_quats[flip] = -raw_diff_quats[flip]

        return Pose(diff_root, diff_hips, Rotation.from_quat(raw_diff_quats))

    @staticmethod
    def lerp(a: Pose, b: Pose, t: float) -> Pose:
        """Linear interpolation for translations and SLERP for rotations."""
        root = (1.0 - t) * a.rootPos + t * b.rootPos
        hips = (1.0 - t) * a.hipPos + t * b.hipPos

        # SciPy Slerp interpolation across time points [0, 1]
        # Note: Slerp expects a Rotation object representing a sequence of rotations.
        # Since 'a.quats' and 'b.quats' might be batches, we handle this by creating
        # a sequence of the two states.

        # We need to construct a Rotation object that represents the start and end
        # stack arrays to fit R.from_quat
        q_stacked = np.stack([a.quats.as_quat(), b.quats.as_quat()])
        r_stacked = Rotation.from_quat(q_stacked)

        slerper = Slerp([0.0, 1.0], r_stacked)
        interpolated_rotations = slerper([t])

        # Slerp returns an array of rotations. We take the one corresponding to 't'
        # Using indexing [0] to extract the Rotation object for time 't'
        quats = interpolated_rotations[0]

        return Pose(root, hips, quats)

    @staticmethod
    def blend(states: List[Pose], weights: np.ndarray) -> Pose:
        """Weighted blend across multiple pose states."""
        weights = np.asarray(weights, dtype=np.float32)

        roots = np.array([s.rootPos for s in states])
        hips_arr = np.array([s.hipPos for s in states])
        quats_arr = np.array([s.quats.as_quat() for s in states])

        root = np.sum(roots * weights[:, np.newaxis], axis=0)
        hips = np.sum(hips_arr * weights[:, np.newaxis], axis=0)

        # Weighted sum of quaternions with normalization
        raw_quats = np.sum(quats_arr * weights[:, np.newaxis, np.newaxis], axis=0)
        norms = np.linalg.norm(raw_quats, axis=-1, keepdims=True)
        norms[norms == 0] = 1.0  # Prevent divide-by-zero
        quats = raw_quats / norms

        return Pose(root, hips, quats)

    # @staticmethod
    # def to_qp(a: PoseData, default_skeleton_p: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    #     """Converts pose data into rigid body position/rotation arrays (qp format)."""
    #     p = default_skeleton_p.copy()
    #     p[0, :] = a.rootPos
    #     p[1, :] = a.hipPos
    #     return a.quats.as_quat(), p
