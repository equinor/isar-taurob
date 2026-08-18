import pytest
from alitra import Bounds, Frame, Map, Position, Positions
from robot_interface.models.exceptions.robot_exceptions import RobotMapException

from isar_taurob.robot.map import TaurobMap

expected_map_name = "test_map"
expected_robot_map = Map(
    name="test_map_robot",
    frame=Frame("robot"),
    reference_positions=Positions(
        positions=[
            Position(x=10, y=20, z=30, frame=Frame("robot")),
            Position(x=40, y=50, z=60, frame=Frame("robot")),
            Position(x=70, y=80, z=90, frame=Frame("robot")),
        ],
        frame=Frame("robot"),
    ),
)
expected_asset_map = Map(
    name="test_map_asset",
    frame=Frame("asset"),
    reference_positions=Positions(
        positions=[
            Position(x=11, y=21, z=31, frame=Frame("asset")),
            Position(x=41, y=51, z=61, frame=Frame("asset")),
            Position(x=71, y=81, z=91, frame=Frame("asset")),
        ],
        frame=Frame("asset"),
    ),
    bounds=Bounds(
        position1=Position(x=5, y=15, z=30, frame=Frame("asset")),
        position2=Position(x=80, y=90, z=100, frame=Frame("asset")),
    ),
)


def test_select_map_by_asset_position(map_handler) -> None:
    position = Position(x=10, y=40, z=60, frame=Frame("asset"))
    assert (
        map_handler.select_map_by_asset_position(position).map_asset
        == expected_asset_map
    )


def test_check_maps_for_overlap(map_handler) -> None:
    map_names = ["default_map", "test_map"]
    assert map_handler._check_maps_for_overlap(map_names)


def test_check_loaded_maps_do_not_overlap(map_handler_loaded_maps) -> None:
    for maphandler in map_handler_loaded_maps:
        map_names = [_map.name for _map in maphandler.maps]

        assert not maphandler._check_maps_for_overlap(map_names)


def test_select_map_by_name(map_handler) -> None:
    taurob_map: TaurobMap = map_handler._select_map_by_name("test_map")
    assert taurob_map.name == expected_map_name
    assert taurob_map.map_robot == expected_robot_map
    assert taurob_map.map_asset == expected_asset_map


def test_select_map_by_asset_position_no_valid_maps(map_handler) -> None:
    point_outside_bounds: Position = Position(x=-2, y=40, z=50, frame=Frame("asset"))

    with pytest.raises(RobotMapException):
        _map: TaurobMap = map_handler.select_map_by_asset_position(point_outside_bounds)


def test_check_map_configs_for_default_asset(map_handler) -> None:
    expected_map_names = ["default_map", "test_map"]

    assert len(map_handler.maps) == len(expected_map_names)
    for map in map_handler.maps:
        assert map.name in expected_map_names
