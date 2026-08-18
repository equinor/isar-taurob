import pytest
from robot_interface.models.exceptions.robot_exceptions import RobotTaskStatusException
from robot_interface.models.mission.task import TaskStatus

from isar_taurob.robot.api.api import API


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
