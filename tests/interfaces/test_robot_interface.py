from unittest import mock

import pytest
from robot_interface.models.exceptions.robot_exceptions import (
    RobotUnreliableDockingStatusException,
)
from robot_interface.models.inspection.inspection import Inspection
from robot_interface.models.mission.mission import Mission
from robot_interface.models.mission.task import TakeImage, TakeThermalVideo, TaskStatus
from robot_interface.models.robots.battery_state import BatteryState

from isar_taurob.robot.api.api import API
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry
from isar_taurob.robotinterface import Robot
from tests.utilities.mock_api_return_values.mock_api import MockAPI
from tests.utilities.mock_models.mock_robot_variables import (
    mock_pose,
    mock_target,
    mock_taurob_pose,
)

successful_instance_id = 123
failed_instance_id = 124
action_id = 1234
thermal_video_instance_id = 125


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(
    TaurobTelemetry,
    "get_robot_battery",
    mock.Mock(return_value=(100, BatteryState.Charging)),
)
@mock.patch.object(
    API,
    "get_sensor_value",
    mock.Mock(
        return_value={
            "name": "robot.state",
            "value": "docked",
            "changed": "2025-08-27T10:07:19.977073+00:00 UTC",
            "version": 591207,
        }
    ),
)
@mock.patch.object(
    API,
    "schedule",
    mock.Mock(return_value=MockAPI.schedule(instance_id=successful_instance_id)),
)
@mock.patch.object(
    API,
    "create_action",
    mock.Mock(return_value=MockAPI.create_action(action_id=action_id)),
)
@mock.patch.object(
    API, "amend_mission", mock.Mock(return_value=MockAPI.amend_mission())
)
def test_initiate_mission() -> None:
    # Arrange
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )
    mission = Mission(
        id="id",
        name="Take one image mission",
        tasks=[TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())],
    )
    taskId = mission.tasks[0].id

    # Act
    robot.initiate_mission(mission)

    # Assert
    assert robot.instance_id == successful_instance_id
    assert robot.action_task_map[taskId].taurob_action_id == action_id


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(
    TaurobTelemetry,
    "get_robot_battery",
    mock.Mock(return_value=(100, BatteryState.Charging)),
)
@mock.patch.object(
    API,
    "get_sensor_value",
    mock.Mock(
        return_value={
            "name": "robot.state",
            "value": "docked",
            "changed": "2025-08-27T10:07:19.977073+00:00 UTC",
            "version": 591207,
        }
    ),
)
@mock.patch.object(
    TaurobTelemetry, "get_robot_pose", mock.Mock(return_value=(mock_taurob_pose()))
)
@mock.patch.object(
    TaurobTelemetry,
    "get_mission_failure_reason",
    mock.Mock(return_value=(("Mission failed due to an error.", None))),
)
@mock.patch.object(
    API,
    "schedule",
    mock.Mock(return_value=MockAPI.schedule(instance_id=successful_instance_id)),
)
@mock.patch.object(
    API,
    "create_action",
    mock.Mock(return_value=MockAPI.create_action(action_id=action_id)),
)
@mock.patch.object(
    API, "amend_mission", mock.Mock(return_value=MockAPI.amend_mission())
)
@mock.patch.object(
    API,
    "get_mission_instance_details",
    mock.Mock(
        return_value=MockAPI.get_mission_instance_details(
            instance_id=successful_instance_id
        )
    ),
)
def test_task_status_where_status_is_successful():
    # Arrange
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )
    mission = Mission(
        id="id",
        name="Take one image mission",
        tasks=[TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())],
    )
    taskId = mission.tasks[0].id
    robot.initiate_mission(mission)

    # Act
    task_status = robot.task_status(taskId)

    # Assert
    assert task_status == TaskStatus.Successful


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(
    TaurobTelemetry,
    "get_robot_battery",
    mock.Mock(return_value=(100, BatteryState.Charging)),
)
@mock.patch.object(
    API,
    "get_sensor_value",
    mock.Mock(
        return_value={
            "name": "robot.state",
            "value": "docked",
            "changed": "2025-08-27T10:07:19.977073+00:00 UTC",
            "version": 591207,
        }
    ),
)
@mock.patch.object(
    TaurobTelemetry, "get_robot_pose", mock.Mock(return_value=(mock_taurob_pose()))
)
@mock.patch.object(
    TaurobTelemetry,
    "get_mission_failure_reason",
    mock.Mock(return_value=(("Mission failed due to an error.", None))),
)
@mock.patch.object(
    API,
    "schedule",
    mock.Mock(return_value=MockAPI.schedule(instance_id=failed_instance_id)),
)
@mock.patch.object(
    API,
    "create_action",
    mock.Mock(return_value=MockAPI.create_action(action_id=action_id)),
)
@mock.patch.object(
    API, "amend_mission", mock.Mock(return_value=MockAPI.amend_mission())
)
@mock.patch.object(
    API,
    "get_mission_instance_details",
    mock.Mock(
        return_value=MockAPI.get_mission_instance_details(
            instance_id=failed_instance_id
        )
    ),
)
def test_task_status_where_status_is_failed():
    # Arrange
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )
    mission = Mission(
        id="id",
        name="Take one image mission",
        tasks=[TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())],
    )
    taskId = mission.tasks[0].id
    robot.initiate_mission(mission)

    # Act
    task_status = robot.task_status(taskId)

    # Assert
    assert task_status == TaskStatus.Failed


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(
    TaurobTelemetry,
    "get_robot_battery",
    mock.Mock(return_value=(100, BatteryState.Charging)),
)
@mock.patch.object(
    TaurobTelemetry, "get_robot_pose", mock.Mock(return_value=(mock_taurob_pose()))
)
@mock.patch.object(
    TaurobTelemetry,
    "get_mission_failure_reason",
    mock.Mock(return_value=(("Mission failed due to an error.", None))),
)
@mock.patch.object(
    API,
    "schedule",
    mock.Mock(return_value=MockAPI.schedule(instance_id=successful_instance_id)),
)
@mock.patch.object(
    API,
    "create_action",
    mock.Mock(return_value=MockAPI.create_action(action_id=action_id)),
)
@mock.patch.object(
    API, "amend_mission", mock.Mock(return_value=MockAPI.amend_mission())
)
@mock.patch.object(
    API,
    "get_mission_instance_details",
    mock.Mock(
        return_value=MockAPI.get_mission_instance_details(
            instance_id=successful_instance_id
        )
    ),
)
@mock.patch.object(
    API,
    "get_sensor_value",
    mock.Mock(
        return_value={
            "name": "robot.state",
            "value": "docked",
            "changed": "2025-08-27T10:07:19.977073+00:00 UTC",
            "version": 591207,
        }
    ),
)
@mock.patch.object(
    API,
    "get_url_data",
    mock.Mock(return_value=MockAPI.get_url_data()),
)
def test_get_inspection():
    # Arrange
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )

    task = TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())
    mission = Mission(
        id="id",
        name="Take one image mission",
        tasks=[task],
    )
    robot.initiate_mission(mission)
    robot.task_status(task.id)

    # Act
    inspection = robot.get_inspection(task)

    # Assert
    assert isinstance(inspection, Inspection)


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(
    TaurobTelemetry,
    "get_robot_battery",
    mock.Mock(return_value=(100, BatteryState.Charging)),
)
@mock.patch.object(
    TaurobTelemetry, "get_robot_pose", mock.Mock(return_value=(mock_taurob_pose()))
)
@mock.patch.object(
    TaurobTelemetry,
    "get_mission_failure_reason",
    mock.Mock(return_value=(("Mission failed due to an error.", None))),
)
@mock.patch.object(
    API,
    "schedule",
    mock.Mock(return_value=MockAPI.schedule(instance_id=thermal_video_instance_id)),
)
@mock.patch.object(
    API,
    "create_action",
    mock.Mock(return_value=MockAPI.create_action(action_id=action_id)),
)
@mock.patch.object(
    API, "amend_mission", mock.Mock(return_value=MockAPI.amend_mission())
)
@mock.patch.object(
    API,
    "get_mission_instance_details",
    mock.Mock(
        return_value=MockAPI.get_mission_instance_details(
            instance_id=thermal_video_instance_id
        )
    ),
)
@mock.patch.object(
    API,
    "get_sensor_value",
    mock.Mock(
        return_value={
            "name": "robot.state",
            "value": "docked",
            "changed": "2025-08-27T10:07:19.977073+00:00 UTC",
            "version": 591207,
        }
    ),
)
@mock.patch.object(
    API,
    "get_url_data",
    mock.Mock(return_value=MockAPI.get_url_data()),
)
def test_get_inspection_on_thermal_video():
    # Arrange
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )

    task = TakeThermalVideo(
        id="id", robot_pose=mock_pose(), target=mock_target(), duration=10
    )
    mission = Mission(
        id="id",
        name="Take one thermal video mission",
        tasks=[task],
    )
    robot.initiate_mission(mission)
    robot.task_status(task.id)

    # Act
    inspection = robot.get_inspection(task)

    # Assert
    assert isinstance(inspection, Inspection)


@mock.patch.object(
    TaurobTelemetry, "get_robot_remote_stop", mock.Mock(return_value=False)
)
@mock.patch.object(
    TaurobTelemetry,
    "get_robot_battery",
    mock.Mock(return_value=(100, BatteryState.Charging)),
)
@mock.patch.object(
    API,
    "get_sensor_value",
    mock.Mock(
        return_value={
            "name": "robot.state",
            "value": "undocked",
            "changed": "2025-08-27T10:07:19.977073+00:00 UTC",
            "version": 591207,
        }
    ),
)
def test_initiate_mission_throws_unreliable_docking_status_exception():
    # Arrange
    robot: Robot = Robot(
        robot_name="William", isar_id="00000000-0000-0000-0000-000000000000"
    )
    task = TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())
    mission = Mission(
        id="id",
        name="Take one image mission",
        tasks=[task],
    )

    # Act and Assert
    with pytest.raises(RobotUnreliableDockingStatusException):
        robot.initiate_mission(mission)
