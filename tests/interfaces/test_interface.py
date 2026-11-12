from datetime import datetime, timedelta
from unittest import mock

import pytz
from robot_interface.models.exceptions.robot_exceptions import (
    RobotTelemetryNoUpdateException,
)
from robot_interface.models.mission.status import RobotStatus
from robot_interface.test_robot_interface import interface_test

from isar_taurob.config import settings
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.request_handler import RequestHandler
from isar_taurob.robot.api.sensor import TAUROB_TIME_FORMAT
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry
from isar_taurob.robotinterface import Robot
from tests.utilities.mock_api_return_values.mock_taurob_api import MockTaurobAPI

mock_recent_snapshot = MockTaurobAPI.get_all_sensor_values()

mock_old_snapshot = MockTaurobAPI.get_all_sensor_values(
    changed=(
        datetime.now().astimezone(tz=pytz.utc)
        - timedelta(seconds=settings.ROBOT_REQUESTS_TIMEOUT + 60)
    ).strftime(TAUROB_TIME_FORMAT)
)


def test_interface() -> None:
    interface_test(
        Robot(robot_name="William", isar_id="00000000-0000-0000-0000-000000000000")
    )


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(API, "_unpack_json", mock.Mock(return_value=mock_recent_snapshot))
@mock.patch.object(API, "get_scheduled_actions", mock.Mock(return_value=[]))
@mock.patch.object(RequestHandler, "get", mock.Mock())
@mock.patch.object(
    TaurobTelemetry, "get_robot_obstacle_status", mock.Mock(return_value=False)
)
def test_robot_status_is_available_when_recent_update() -> None:
    # Check pre-condition
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )
    status: RobotStatus = robot.robot_status()
    assert status is RobotStatus.Offline

    robot.telemetry.refresh_sensor_cache()
    robot._get_pressure_telemetry()

    # Check that robot is set to available
    newStatus: RobotStatus = robot.robot_status()
    assert newStatus is RobotStatus.Available


@mock.patch.object(API, "_unpack_json", mock.Mock(return_value=mock_old_snapshot))
@mock.patch.object(API, "get_scheduled_actions", mock.Mock(return_value=[]))
@mock.patch.object(RequestHandler, "get", mock.Mock())
@mock.patch.object(
    TaurobTelemetry, "get_robot_obstacle_status", mock.Mock(return_value=False)
)
def test_robot_status_is_offline_when_old_update() -> None:
    # Check pre-condition
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )
    status: RobotStatus = robot.robot_status()
    assert status is RobotStatus.Offline

    robot.telemetry.refresh_sensor_cache()
    try:
        robot._get_pressure_telemetry()
    except RobotTelemetryNoUpdateException:
        pass  # Expected exception for outdated telemetry

    # Check that robot is still offline
    newStatus: RobotStatus = robot.robot_status()
    assert newStatus is RobotStatus.Offline
