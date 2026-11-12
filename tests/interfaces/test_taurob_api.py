import datetime
from http import HTTPStatus

import pytest
from requests.exceptions import RequestException
from robot_interface.models.exceptions.robot_exceptions import (
    RobotAPIException,
    RobotCommunicationException,
)

from isar_taurob.config import settings
from isar_taurob.robot.api.request_handler import RequestHandler
from isar_taurob.robot.api.sensor import SensorName, SensorValue
from tests.utilities.mock_api_return_values.mock_taurob_api import MockTaurobAPI


class MockRequests:
    def __init__(self, json_data=None, content=None, status_code=HTTPStatus.OK):
        self.status_code = status_code
        self.json_data = json_data
        self.content = content
        settings.ROBOT_NUMBER_OF_JOINTS = 2

    def json(self):
        return self.json_data

    def content(self):
        return self.content


@pytest.mark.parametrize(
    "mock_return, mission_id, step_list, expected_response",
    [
        (
            [MockRequests(json_data=MockTaurobAPI.schedule("success"))],
            3,
            {"steps": [300, 302]},
            232,
        )
    ],
)
def test_schedule_mission(
    mocker, taurob_robot_api, mock_return, expected_response, mission_id, step_list
):
    mocker.patch.object(
        RequestHandler,
        "post",
        side_effect=mock_return,
    )
    response = taurob_robot_api.schedule(mission_id, step_list)
    assert response == expected_response


@pytest.mark.parametrize(
    "mock_return, expected_response",
    [
        (
            [MockRequests(json_data=MockTaurobAPI.get_scheduled_missions("success"))],
            [232, 195],
        ),
        (RequestException(), None),
        (
            [
                MockRequests(
                    json_data=MockTaurobAPI.get_scheduled_missions("success_empty")
                )
            ],
            [],
        ),
    ],
)
def test_get_scheduled_mission(
    mocker, taurob_robot_api, mock_return, expected_response
):
    mocker.patch.object(
        RequestHandler,
        "get",
        side_effect=mock_return,
    )
    taurob_robot_api.robot_name = "ltdata"
    try:
        response = taurob_robot_api.get_scheduled_missions()
    except RobotCommunicationException:
        response = None
    assert response == expected_response


@pytest.mark.parametrize(
    "mock_return, expected_response, instance_id",
    [
        (
            [MockRequests(json_data=MockTaurobAPI.get_mission_details(232))],
            MockTaurobAPI.get_mission_details(232),
            232,
        ),
        (
            [MockRequests(json_data=MockTaurobAPI.get_mission_details(-1))],
            [],
            232,
        ),
        (RequestException(), None, 232),
    ],
)
def test_get_mission_instance_details(
    mocker, taurob_robot_api, mock_return, expected_response, instance_id
):
    mocker.patch.object(
        RequestHandler,
        "get",
        side_effect=mock_return,
    )
    try:
        response = taurob_robot_api.get_mission_instance_details(instance_id)
    except RobotCommunicationException:
        response = None
    assert response == expected_response


@pytest.mark.parametrize(
    "mock_return_get, mock_return_delete, expected_response",
    [
        (
            [MockRequests(json_data=([]))],
            [{"dummy"}, {"dummy"}],
            True,
        ),
        (
            RequestException(),
            [{"dummy"}, {"dummy"}],
            False,
        ),
        (
            [MockRequests(json_data=MockTaurobAPI.get_scheduled_missions("success"))],
            [{"dummy"}, {"dummy"}],
            True,
        ),
    ],
)
def test_delete_all_mission_instances(
    mocker, taurob_robot_api, mock_return_get, mock_return_delete, expected_response
):
    mocker.patch.object(
        RequestHandler,
        "get",
        side_effect=mock_return_get,
    )
    mocker.patch.object(
        RequestHandler,
        "delete",
        side_effect=mock_return_delete,
    )
    try:
        taurob_robot_api.delete_all_mission_instances()
        response = True
    except RobotCommunicationException:
        response = False
    assert response == expected_response


@pytest.mark.parametrize(
    "sensor_name, mock_return, expected_output",
    [
        (
            "pose.base",
            MockRequests(json_data=MockTaurobAPI.get_sensor_value("pose.base")),
            MockTaurobAPI.get_sensor_value("pose.base"),
        ),
        (
            "pose.arm",
            MockRequests(json_data=MockTaurobAPI.get_sensor_value("pose.arm")),
            MockTaurobAPI.get_sensor_value("pose.arm"),
        ),
        (
            "pose.total_distance",
            MockRequests(
                json_data=MockTaurobAPI.get_sensor_value("pose.total_distance")
            ),
            MockTaurobAPI.get_sensor_value("pose.total_distance"),
        ),
    ],
)
def test_get_sensor_value(
    mocker, taurob_robot_api, sensor_name, mock_return, expected_output
):
    mocker.patch.object(
        RequestHandler,
        "get",
        return_value=mock_return,
    )
    try:
        sensor_value = taurob_robot_api.get_sensor_value(sensor_name)
    except RobotCommunicationException:
        sensor_value = None
    assert sensor_value == SensorValue.model_validate(expected_output)


@pytest.mark.parametrize(
    "sensor_name, raises_exception",
    [
        (
            "pose.base",
            RobotCommunicationException,
        ),
        (
            "pose.base",
            RobotAPIException,
        ),
    ],
)
def test_get_sensor_value_raises_exception(
    mocker,
    taurob_robot_api,
    sensor_name,
    raises_exception,
):
    mocker.patch.object(
        RequestHandler,
        "get",
        side_effect=raises_exception(error_description="Random error description"),
    )

    with pytest.raises(raises_exception):
        taurob_robot_api.get_sensor_value(sensor_name=sensor_name)


def test_get_all_sensor_values_returns_snapshot_keyed_by_name(mocker, taurob_robot_api):
    mocker.patch.object(
        RequestHandler,
        "get",
        return_value=MockRequests(json_data=MockTaurobAPI.get_all_sensor_values()),
    )

    snapshot = taurob_robot_api.get_all_sensor_values()

    for sensor_name in SensorName:
        assert sensor_name in snapshot, f"{sensor_name} missing from snapshot"
        assert snapshot[sensor_name].name == sensor_name

    assert snapshot[SensorName.BATTERY_LEVEL].value == 10
    assert snapshot[SensorName.ROBOT_STATE].value == "idle"


def test_get_all_sensor_values_handles_null_values(mocker, taurob_robot_api):
    mocker.patch.object(
        RequestHandler,
        "get",
        return_value=MockRequests(json_data=MockTaurobAPI.get_all_sensor_values()),
    )

    snapshot = taurob_robot_api.get_all_sensor_values()

    assert snapshot["robot.disable_vps"].value is None
    assert snapshot["robot.disable_vps"].changed is None


def test_get_all_sensor_values_skips_malformed_entries(mocker, taurob_robot_api):
    snapshot_with_garbage = MockTaurobAPI.get_all_sensor_values() + [
        "not a sensor entry",
        {"value": "entry without a name"},
        None,
    ]
    mocker.patch.object(
        RequestHandler,
        "get",
        return_value=MockRequests(json_data=snapshot_with_garbage),
    )

    snapshot = taurob_robot_api.get_all_sensor_values()

    for sensor_name in SensorName:
        assert sensor_name in snapshot


@pytest.mark.parametrize(
    "read_sensors",
    [
        lambda api: api.get_all_sensor_values(),
        lambda api: api.get_sensor_value(SensorName.POSE_BASE),
    ],
)
def test_sensor_reads_fail_without_retrying(mocker, taurob_robot_api, read_sensors):
    request = mocker.patch.object(
        RequestHandler,
        "get",
        side_effect=RequestException("Connection reset"),
    )

    with pytest.raises(RobotCommunicationException):
        read_sensors(taurob_robot_api)

    assert request.call_count == 1


def test_get_all_sensor_values_times_out_after_refresh_interval(
    mocker, taurob_robot_api
):
    request = mocker.patch.object(
        RequestHandler,
        "get",
        return_value=MockRequests(json_data=MockTaurobAPI.get_all_sensor_values()),
    )

    taurob_robot_api.get_all_sensor_values()

    assert (
        request.call_args.kwargs["request_timeout"] == settings.SENSOR_REFRESH_INTERVAL
    )


def test_sensor_value_is_older_than():
    changed = datetime.datetime.now(datetime.UTC) - datetime.timedelta(minutes=5)
    sensor_value = SensorValue(name="test", value=1, changed=changed)

    assert sensor_value.is_older_than(datetime.timedelta(minutes=1))
    assert not sensor_value.is_older_than(datetime.timedelta(minutes=10))


def test_sensor_value_without_changed_is_stale():
    sensor_value = SensorValue(name="test", value=1)

    assert sensor_value.is_older_than(datetime.timedelta(days=365))
