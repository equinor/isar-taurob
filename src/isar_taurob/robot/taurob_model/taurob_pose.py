from dataclasses import dataclass

import numpy as np
from alitra import Frame, Orientation, Pose, Position


@dataclass
class TaurobPose:
    x: float
    y: float
    theta: float
    map_name: str

    def to_pose(self) -> Pose:
        robot_frame: Frame = Frame("robot")
        orientation: Orientation = Orientation.from_euler_array(
            np.array([self.theta, 0, 0]), frame=robot_frame
        )

        return Pose(
            position=Position(x=self.x, y=self.y, z=0, frame=robot_frame),
            orientation=orientation,
            frame=robot_frame,
        )
