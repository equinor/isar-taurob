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
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry
from isar_taurob.robotinterface import Robot

mock_recent_pressure_update = {
    "name": "sensor.pcu.pressure_1",
    "value": 12.34,
    "changed": datetime.now().astimezone(tz=pytz.utc).strftime(API.TAUROB_TIME_FORMAT),
    "version": 0,
}

mock_old_pressure_update = {
    "name": "sensor.pcu.pressure_1",
    "value": 12.34,
    "changed": (
        datetime.now().astimezone(tz=pytz.utc)
        - timedelta(seconds=settings.ROBOT_REQUESTS_TIMEOUT + 60)
    ).strftime(API.TAUROB_TIME_FORMAT),
    "version": 0,
}


def test_interface() -> None:
    interface_test(
        Robot(robot_name="William", isar_id="00000000-0000-0000-0000-000000000000")
    )


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(
    API, "_unpack_json", mock.Mock(return_value=mock_recent_pressure_update)
)
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

    # Get a status update of pressure
    robot._get_pressure_telemetry("test_isar_id", "test_robot_name")

    # Check that robot is set to available
    newStatus: RobotStatus = robot.robot_status()
    assert newStatus is RobotStatus.Available


@mock.patch.object(
    API, "_unpack_json", mock.Mock(return_value=mock_old_pressure_update)
)
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

    # Get a status update of pressure
    try:
        robot._get_pressure_telemetry("test_isar_id", "test_robot_name")
    except RobotTelemetryNoUpdateException:
        pass  # Expected exception for outdated telemetry

    # Check that robot is still offline
    newStatus: RobotStatus = robot.robot_status()
    assert newStatus is RobotStatus.Offline
