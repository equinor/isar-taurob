from alitra import Frame, Orientation, Pose, Position
from robot_interface.models.mission.task import (
    ReturnToHome,
    TakeImage,
    TakeThermalImage,
    TakeThermalVideo,
)

_ROBOT_FRAME = Frame("robot")
_DEFAULT_POSITION = Position(x=1, y=1, z=1, frame=_ROBOT_FRAME)
_DEFAULT_ORIENTATION = Orientation(
    x=0, y=0, z=0.7071068, w=0.7071068, frame=_ROBOT_FRAME
)
_DEFAULT_TARGET = Position(x=10, y=10, z=10, frame=_ROBOT_FRAME)


class MockStep:
    @staticmethod
    def drive_to(
        position: Position = _DEFAULT_POSITION,
        orientation: Orientation = _DEFAULT_ORIENTATION,
    ) -> ReturnToHome:
        return ReturnToHome(
            pose=Pose(position=position, orientation=orientation, frame=_ROBOT_FRAME)
        )

    @staticmethod
    def take_image_in_coordinate_direction(
        target: Position = _DEFAULT_TARGET,
    ) -> TakeImage:
        return TakeImage(target=target)

    @staticmethod
    def take_thermal_image(
        target: Position = _DEFAULT_TARGET,
    ) -> TakeThermalImage:
        return TakeThermalImage(target=target)

    @staticmethod
    def take_thermal_video(
        target: Position = _DEFAULT_TARGET,
        duration: int = 10,
    ) -> TakeThermalVideo:
        return TakeThermalVideo(target=target, duration=duration)
