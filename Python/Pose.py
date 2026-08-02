from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
from typing import List

from utils.quaternions import blend_quaternions


class Pose:
    """
    Represents skeletal poses with root translation, hip translation,
    and joint rotation quaternions.
    """

    rootPos: np.ndarray
    hipPos: np.ndarray
    quats: np.ndarray

    def __init__(self, root: np.ndarray, hips: np.ndarray, quats: np.ndarray):
        """
        :param root: Vector3 root position (..., 3)
        :param hips: Vector3 hips position (..., 3)
        :param quats: Joint rotations quaternions (..., num_bones, 4)
        """
        assert root.shape[-1] == 3, "Root position must be a 3D vector."
        assert hips.shape[-1] == 3, "Hips position must be a 3D vector."
        assert root.shape[:-1] == hips.shape[:-1] and root.shape[:-1] == quats.shape[:-2], "Batch sizes must match."
        assert len(root.shape) > 1, "Root position must have at least 2 dimensions (batch, 3)."

        self.rootPos = np.asarray(root, dtype=np.float32)
        self.hipPos = np.asarray(hips, dtype=np.float32)
        self.quats = quats

    @classmethod
    def from_array(cls, pose: np.ndarray) -> Pose:
        """Unpacks a flat pose tensor into a PoseData instance."""
        root = pose[..., 0, :3].copy()
        hips = pose[..., 1, :3].copy()
        quats = pose[..., 2:, :].copy()
        if len(root.shape) == 1:
            root = np.expand_dims(root, axis=0)
            hips = np.expand_dims(hips, axis=0)
            quats = np.expand_dims(quats, axis=0)

        return Pose(root, hips, quats)

    def pack(self) -> np.ndarray:
        """Packs the PoseData back into a single flat array representation."""
        # Get raw quaternion array to determine shape
        num_bones = self.quats.shape[-2]
        batch_shape = self.quats.shape[:-2]

        result = np.zeros((*batch_shape, num_bones + 2, 4), dtype=np.float32)
        result[..., 0, :3] = self.rootPos
        result[..., 1, :3] = self.hipPos
        result[..., 2:, :] = self.quats
        return result

    def add(self, other: Pose) -> Pose:
        """
        Adds a velocity/delta pose to this pose.
        (Calculates relative transformation for root and composes rotations).
        """
        # Extract root rotations (assuming index 0 is the root bone)
        # Using slicing [0:1] to maintain shape or indexing [0] based on usage
        # We assume the user wants the first rotation in the sequence.

        r_x0: Rotation = Rotation.from_quat(self.quats[..., 0, :])
        r_v0: Rotation = Rotation.from_quat(other.quats[..., 0, :])

        # Multiply rotation and rotate offset vector
        new_root = self.rootPos + r_x0.apply(other.rootPos)

        # Hips vector addition
        new_hips = self.hipPos + other.hipPos

        # Joint rotations composition
        new_quats = np.array(Rotation.as_quat(
            Rotation.from_quat(other.quats) *
            Rotation.from_quat(self.quats)
        ))

        return Pose(new_root, new_hips, new_quats)

    def __sub__(self, other: Pose) -> Pose:
        """
        Subtracts a base pose 'other' from 'self' to get the delta pose.
        """
        # Inverse root rotation of 'other' (b)

        r_b0_inv: Rotation = Rotation.from_quat(other.quats[..., 0, :]).inv()

        # Relative root position & rotation
        diff_root = np.array(
            r_b0_inv.apply(self.rootPos - other.rootPos)
        )

        # Hips vector difference
        diff_hips = self.hipPos - other.hipPos

        # Relative joint rotations: inverse(b) * a
        diff_quats = np.array(Rotation.as_quat(
            Rotation.from_quat(other.quats).inv() *
            Rotation.from_quat(self.quats)
        ))
        # Because we use [x, y, z, w], the scalar part 'w' is now at index -1
        flip = diff_quats[..., -1] < 0
        diff_quats[flip] = -diff_quats[flip]

        return Pose(diff_root, diff_hips, diff_quats)

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
        q_stacked = np.stack([a.quats, b.quats])
        r_stacked = Rotation.from_quat(q_stacked)

        slerper = Slerp([0.0, 1.0], r_stacked)
        interpolated_rotations = slerper([t])

        # Slerp returns an array of rotations. We take the one corresponding to 't'
        # Using indexing [0] to extract the Rotation object for time 't'
        quats = np.array(interpolated_rotations[:1].as_quat())

        return Pose(root, hips, quats)

    @staticmethod
    def blend(poses: Pose, weights: np.ndarray) -> Pose:
        """Weighted blend across multiple pose states."""
        weights = np.asarray(weights, dtype=np.float32)

        # poses.rootPos (..., 3), poses.hipPos (..., 3), poses.quats (..., num_bones, 4)
        roots = np.sum(poses.rootPos * weights[..., np.newaxis], axis=0)
        hips = np.sum(poses.hipPos * weights[..., np.newaxis], axis=0)
        quats = blend_quaternions(poses.quats, weights, axis=0)

        roots = np.expand_dims(roots, axis=0)
        hips = np.expand_dims(hips, axis=0)
        quats = np.expand_dims(quats, axis=0)

        return Pose(roots, hips, quats)

    def scaled(self, scale_factor: float) -> Pose:
        """Scales the pose by a given factor."""
        scaled_root = self.rootPos * scale_factor
        scaled_hips = self.hipPos * scale_factor
        axis_angle = Rotation.from_quat(self.quats).as_rotvec()
        scaled_quats = np.array(Rotation.from_rotvec(axis_angle * scale_factor).as_quat())

        return Pose(scaled_root, scaled_hips, scaled_quats)

    @staticmethod
    def concatenate(poses: List[Pose]) -> Pose:
        """Concatenates a list of Pose instances along the batch dimension."""
        roots = np.concatenate([p.rootPos for p in poses], axis=0)
        hips = np.concatenate([p.hipPos for p in poses], axis=0)
        quats = np.concatenate([p.quats for p in poses], axis=0)
        return Pose(roots, hips, quats)

