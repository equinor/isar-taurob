import json
from dataclasses import asdict

import pytest
from pytest_mock import MockerFixture
from robot_interface.models.exceptions.robot_exceptions import RobotTaskStatusException
from robot_interface.models.mission.task import TakeAcousticMeasurement, TaskStatus

from isar_taurob.config import settings
from isar_taurob.robot.api.acoustic_metadata import ACOUSTIC_MEASUREMENT_DURATION
from isar_taurob.robot.api.api import API, ActionType
from isar_taurob.robot.api.measurement_metadata import MeasurementMetadata
from isar_taurob.robot.scheduler.scheduler import Scheduler
from tests.utilities.mock_models.mock_robot_variables import mock_taurob_pose


@pytest.mark.parametrize("sensitivity", ["low", "medium", "high"])
def test_acoustic_measurement_request(
    scheduler: Scheduler,
    acoustic_task: TakeAcousticMeasurement,
    mocker: MockerFixture,
    sensitivity: str,
) -> None:
    mocker.patch.object(settings, "ACOUSTIC_SENSITIVITY", sensitivity)
    metadata = scheduler.get_measurement_metadata(acoustic_task)
    expected = {
        "sensor_name": "camera_acoustic",
        "result_type": "leak-detection",
        "result_format": "video/mp4",
        "duration": ACOUSTIC_MEASUREMENT_DURATION,
        "sensitivity": sensitivity,
        "lights": False,
    }
    assert asdict(metadata, dict_factory=MeasurementMetadata.dict_factory) == expected

    response = mocker.Mock()
    response.json.return_value = {"ActionID": 1234}
    post = mocker.patch.object(
        scheduler.robot_api.request_handler, "post", return_value=response
    )
    scheduler.robot_api.create_action(
        acoustic_task.id,
        ActionType.measure,
        mock_taurob_pose(),
        "test-map",
        metadata=metadata,
    )
    body = post.call_args.kwargs["json_body"]
    assert json.loads(body["Metadata"]) == {"measurement": expected}


@pytest.mark.parametrize(
    "instance_id, action_id, mock_return, expected_output",
    [
        (
            1,
            123,
            {
                "get_mission_instance_details": (
                    [
                        {
                            "instance_id": 0,
                            "status": "scheduled",
                            "measurement_results": [
                                {"url": "string", "media_type": "string"}
                            ],
                            "mission_step": {
                                "mission_id": 0,
                                "step_number": 0,
                                "step_id": 123,
                                "step_type": "string",
                            },
                        }
                    ]
                ),
            },
            TaskStatus.NotStarted,
        ),
        (
            1,
            124,
            {
                "get_mission_instance_details": (
                    [
                        {
                            "instance_id": 0,
                            "status": "successful",
                            "measurement_results": [
                                {"url": "string", "media_type": "string"}
                            ],
                            "mission_step": {
                                "mission_id": 0,
                                "step_number": 0,
                                "step_id": 124,
                                "step_type": "string",
                            },
                        }
                    ]
                ),
            },
            TaskStatus.Successful,
        ),
        (
            1,
            125,
            {
                "get_mission_instance_details": (
                    [
                        {
                            "instance_id": 0,
                            "status": "cancelled",
                            "measurement_results": [
                                {"url": "string", "media_type": "string"}
                            ],
                            "mission_step": {
                                "mission_id": 0,
                                "step_number": 0,
                                "step_id": 125,
                                "step_type": "string",
                            },
                        }
                    ]
                ),
            },
            TaskStatus.Failed,
        ),
        (
            1,
            126,
            {
                "get_mission_instance_details": (
                    [
                        {
                            "instance_id": 0,
                            "status": "started",
                            "measurement_results": [
                                {"url": "string", "media_type": "string"}
                            ],
                            "mission_step": {
                                "mission_id": 0,
                                "step_number": 0,
                                "step_id": 126,
                                "step_type": "string",
                            },
                        }
                    ]
                ),
            },
            TaskStatus.InProgress,
        ),
    ],
)
def test_is_scheduled_task_status(
    scheduler, mocker, instance_id, action_id, mock_return, expected_output
) -> None:
    mocker.patch.object(
        API,
        "get_mission_instance_details",
        return_value=mock_return["get_mission_instance_details"],
    )
    task_status: TaskStatus
    task_status = scheduler.get_task_status(instance_id, action_id)

    assert task_status is expected_output


@pytest.mark.parametrize(
    "instance_id, action_id, mock_return, expected_output",
    [
        (
            1,
            0,
            {
                "get_mission_instance_details": (
                    [
                        {
                            "instance_id": 0,
                            "status": "scheduled",
                            "measurement_results": [
                                {"url": "string", "media_type": "string"}
                            ],
                            "mission_step": {
                                "mission_id": 0,
                                "step_number": 0,
                                "step_id": 123,
                                "step_type": "string",
                            },
                        }
                    ]
                ),
            },
            None,
        )
    ],
)
def test_invalid_action_id(
    scheduler, mocker, instance_id, action_id, mock_return, expected_output
) -> None:
    mocker.patch.object(
        API,
        "get_mission_instance_details",
        return_value=mock_return["get_mission_instance_details"],
    )
    task_status: TaskStatus
    try:
        task_status = scheduler.get_task_status(instance_id, action_id)
    except RobotTaskStatusException:
        task_status = None

    assert task_status is expected_output
