import numpy as np
from Skeleton import Skeleton


class PoseSet:
    def __init__(self,
                 skeleton: Skeleton,
                 frame_time: float,
                 local_pos,
                 local_quats,
                 foot_contacts,
                 local_vel,
                 local_angular_vel,
                 clips,
                 ):
        self.skeleton: Skeleton
        self.frameTime: float
        self.pos: np.ndarray
        self.quats: np.ndarray
        self.local_velocities: np.ndarray
        self.local_angular_velocities: np.ndarray
        self.foot_contacts: np.ndarray
        self.clips: list
        """
        :param quats: local quaternions tensor
        :param pos: local positions tensor
        :param skeleton: skeleton object
        :param frameTime: time per frame
        """
        self.local_positions = local_pos
        self.local_rotations = local_quats
        self.frameTime = frame_time
        self.foot_contacts = foot_contacts
        self.local_velocities = local_vel
        self.local_angular_velocities = local_angular_vel
        self.skeleton = skeleton
        self.clips = clips
