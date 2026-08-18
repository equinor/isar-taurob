from datetime import datetime

import pytest
import pytz
from robot_interface.models.exceptions.robot_exceptions import RobotAPIException

from isar_taurob.robot.api.api import API
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose


def test_get_robot_pose(telemetry_service, mocker):
    mock_return = {
        "name": "pose.base",
        "value": '{"floor":"test_map","theta":-3.0683345109302906,"x":16.021346168155628,"y":-6.737769321942849}',
        "changed": datetime.now()
        .astimezone(tz=pytz.utc)
        .strftime(API.TAUROB_TIME_FORMAT),
        "version": 24041,
    }
    mocker.patch.object(API, "get_sensor_value", return_value=mock_return)
    expected_output = TaurobPose(
        x=16.021346168155628,
        y=-6.737769321942849,
        theta=-3.0683345109302906,
        map_name="test_map",
    )
    robot_pose = telemetry_service.get_robot_pose()
    assert robot_pose == expected_output


def test_exceptions(telemetry_service, mocker):
    mock_return = {
        "name": "pose.base",
        "value": '["this is not a dict"]',
        "changed": datetime.now()
        .astimezone(tz=pytz.utc)
        .strftime(API.TAUROB_TIME_FORMAT),
        "version": 24041,
    }
    mocker.patch.object(API, "get_sensor_value", return_value=mock_return)
    with pytest.raises(RobotAPIException):
        _ = telemetry_service.get_robot_pose()
