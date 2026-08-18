import math

from alitra import Frame, Orientation, Pose, Position

from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose

_ASSET_FRAME = Frame("asset")
_DEFAULT_POSITION = Position(x=10, y=20, z=40, frame=_ASSET_FRAME)
_DEFAULT_ORIENTATION = Orientation(
    x=0, y=0, z=0.7071068, w=0.7071068, frame=_ASSET_FRAME
)
_DEFAULT_TARGET_POSITION = Position(x=12, y=22, z=42, frame=_ASSET_FRAME)


def mock_pose(
    position: Position = _DEFAULT_POSITION,
    orientation: Orientation = _DEFAULT_ORIENTATION,
) -> Pose:
    return Pose(position=position, orientation=orientation, frame=_ASSET_FRAME)


def mock_target(
    position: Position = _DEFAULT_TARGET_POSITION,
) -> Position:
    return position


def mock_taurob_pose() -> TaurobPose:
    return TaurobPose(
        x=15,
        y=25,
        theta=math.pi / 2.0,
        map_name="test_map",
    )
