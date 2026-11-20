from collections.abc import Callable
from datetime import UTC, datetime, timedelta
from unittest import mock

import pytest
from pytest_mock import MockerFixture
from robot_interface.models.exceptions.robot_exceptions import (
    RobotAlreadyHomeException,
    RobotDockingStatusMismatchException,
    RobotInfeasibleMissionException,
)
from robot_interface.models.inspection.inspection import Inspection
from robot_interface.models.mission.mission import Mission
from robot_interface.models.mission.status import RobotStatus
from robot_interface.models.mission.task import TakeImage, TakeThermalVideo, TaskStatus

from isar_taurob.config import settings
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.sensor import SensorName, SensorValue
from isar_taurob.robotinterface import Robot
from tests.conftest import build_snapshot
from tests.utilities.mock_api_return_values.mock_api import MockAPI
from tests.utilities.mock_api_return_values.mock_taurob_api import MockTaurobAPI
from tests.utilities.mock_models.mock_robot_variables import mock_pose, mock_target

successful_instance_id = 123
failed_instance_id = 124
action_id = 1234
thermal_video_instance_id = 125
dock_action_id = settings.WALLBOX_DOCK_ACTION_ID


@pytest.fixture()
def mocked_failure_reason(mocker: MockerFixture) -> mock.MagicMock:
    """get_mission_failure_reason bypasses the sensor cache, so it needs its own mock."""
    return mocker.patch.object(
        API,
        "get_sensor_value",
        return_value=SensorValue.model_validate(
            MockTaurobAPI.get_sensor_value("last_mission_failure_reason")
        ),
    )


@pytest.fixture()
def docked_robot(
    robot: Robot,
    mocked_sensors: mock.MagicMock,
    sensor_overrides: dict,
    mocker: MockerFixture,
) -> Robot:
    """A robot that is docked, localized and able to build a mission."""
    sensor_overrides[SensorName.ROBOT_STATE] = "docked"
    mocker.patch.object(
        API, "create_action", return_value=MockAPI.create_action(action_id=action_id)
    )
    mocker.patch.object(API, "amend_mission", return_value=MockAPI.amend_mission())
    return robot


@pytest.fixture()
def robot_running_mission(
    docked_robot: Robot, mocked_failure_reason: mock.MagicMock, mocker: MockerFixture
) -> Callable[[int], Robot]:
    """Returns a factory that arms the robot with a given mission instance id."""

    def _running(instance_id: int) -> Robot:
        mocker.patch.object(
            API, "schedule", return_value=MockAPI.schedule(instance_id=instance_id)
        )
        mocker.patch.object(
            API,
            "get_mission_instance_details",
            return_value=MockAPI.get_mission_instance_details(instance_id=instance_id),
        )
        return docked_robot

    return _running


def take_image_mission() -> Mission:
    return Mission(
        id="id",
        name="Take one image mission",
        tasks=[TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())],
    )


def test_initiate_mission(robot_running_mission: Callable[[int], Robot]) -> None:
    # Arrange
    robot: Robot = robot_running_mission(successful_instance_id)
    mission: Mission = take_image_mission()
    taskId = mission.tasks[0].id

    # Act
    robot.initiate_mission(mission)

    # Assert
    assert robot.instance_id == successful_instance_id
    assert robot.action_task_map[taskId].taurob_action_id == action_id


def test_initiate_mission_is_rejected_when_not_localized_and_not_docked(
    robot_running_mission: Callable[[int], Robot], sensor_overrides: dict
) -> None:
    # Arrange
    robot: Robot = robot_running_mission(successful_instance_id)
    sensor_overrides[SensorName.ROBOT_STATE] = "undocked"
    sensor_overrides[SensorName.LOCALIZATION_RUNNING] = False

    # Act and Assert
    with pytest.raises(RobotInfeasibleMissionException):
        robot.initiate_mission(take_image_mission())


def test_initiate_mission_is_allowed_when_docked_but_not_localized(
    robot_running_mission: Callable[[int], Robot], sensor_overrides: dict
) -> None:
    # Arrange
    robot: Robot = robot_running_mission(successful_instance_id)
    sensor_overrides[SensorName.LOCALIZATION_RUNNING] = False

    # Act
    robot.initiate_mission(take_image_mission())

    # Assert
    assert robot.instance_id == successful_instance_id


@pytest.mark.parametrize(
    "is_docked, expected_action_ids",
    [
        (True, [dock_action_id, action_id]),
        (False, [action_id]),
    ],
)
def test_create_actions_only_adds_initial_action_when_docked(
    robot: Robot,
    mocked_sensors: mock.MagicMock,
    sensor_overrides: dict,
    mocker: MockerFixture,
    is_docked: bool,
    expected_action_ids: list[int],
) -> None:
    # Arrange
    sensor_overrides[SensorName.ROBOT_STATE] = "docked" if is_docked else "undocked"
    create_action = mocker.patch.object(
        API, "create_action", return_value=MockAPI.create_action(action_id=action_id)
    )
    task = TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())

    # Act
    action_ids = robot._create_actions([task])

    # Assert
    assert action_ids == expected_action_ids
    create_action.assert_called_once()


@pytest.mark.parametrize(
    "steps, is_docked, expected_action_ids",
    [
        ([(action_id, "measure")], False, [action_id]),
        (
            [(dock_action_id, "dock"), (action_id, "measure")],
            False,
            [action_id],
        ),
        (
            [(dock_action_id, "dock"), (action_id, "measure")],
            True,
            [dock_action_id, action_id],
        ),
    ],
)
def test_resume_only_adds_initial_action_when_docked(
    robot: Robot,
    mocked_sensors: mock.MagicMock,
    sensor_overrides: dict,
    mocker: MockerFixture,
    steps: list[tuple[int, str]],
    is_docked: bool,
    expected_action_ids: list[int],
) -> None:
    # Arrange
    sensor_overrides[SensorName.ROBOT_STATE] = "docked" if is_docked else "undocked"
    mocker.patch.object(
        API,
        "get_mission_instance_details",
        return_value=MockTaurobAPI.get_cancelled_mission_details(steps),
    )
    mocker.patch.object(API, "amend_mission", return_value=MockAPI.amend_mission())
    schedule = mocker.patch.object(
        API,
        "schedule",
        return_value=MockAPI.schedule(instance_id=successful_instance_id),
    )
    robot.paused_mission_instance_id = failed_instance_id

    # Act
    robot.resume()

    # Assert
    assert schedule.call_args.kwargs["step_list"]["steps"] == expected_action_ids


def test_task_status_where_status_is_successful(
    robot_running_mission: Callable[[int], Robot],
) -> None:
    # Arrange
    robot: Robot = robot_running_mission(successful_instance_id)
    mission: Mission = take_image_mission()
    taskId = mission.tasks[0].id
    robot.initiate_mission(mission)

    # Act
    task_status = robot.task_status(taskId)

    # Assert
    assert task_status == TaskStatus.Successful


def test_task_status_where_status_is_failed(
    robot_running_mission: Callable[[int], Robot],
) -> None:
    # Arrange
    robot: Robot = robot_running_mission(failed_instance_id)
    mission: Mission = take_image_mission()
    taskId = mission.tasks[0].id
    robot.initiate_mission(mission)

    # Act
    task_status = robot.task_status(taskId)

    # Assert
    assert task_status == TaskStatus.Failed


def test_get_inspections(
    robot_running_mission: Callable[[int], Robot], mocker: MockerFixture
) -> None:
    # Arrange
    robot: Robot = robot_running_mission(successful_instance_id)
    mocker.patch.object(API, "get_url_data", return_value=MockAPI.get_url_data())

    task = TakeImage(id="id", robot_pose=mock_pose(), target=mock_target())
    mission = Mission(id="id", name="Take one image mission", tasks=[task])
    robot.initiate_mission(mission)
    robot.task_status(task.id)

    # Act
    inspections = robot.get_inspections(task)
    inspection = inspections[0]

    # Assert
    assert isinstance(inspection, Inspection)


def test_get_inspection_on_thermal_video(
    robot_running_mission: Callable[[int], Robot], mocker: MockerFixture
) -> None:
    # Arrange
    robot: Robot = robot_running_mission(thermal_video_instance_id)
    mocker.patch.object(API, "get_url_data", return_value=MockAPI.get_url_data())

    task = TakeThermalVideo(
        id="id", robot_pose=mock_pose(), target=mock_target(), duration=10
    )
    mission = Mission(id="id", name="Take one thermal video mission", tasks=[task])
    robot.initiate_mission(mission)
    robot.task_status(task.id)

    # Act
    inspections = robot.get_inspections(task)
    inspection = inspections[0]

    # Assert
    assert isinstance(inspection, Inspection)


def test_initiate_return_home_throws_unreliable_docking_status_exception(
    robot: Robot, mocked_sensors: mock.MagicMock, sensor_overrides: dict
) -> None:
    # Arrange: charging but not docked is an inconsistent docking status
    sensor_overrides[SensorName.ROBOT_STATE] = "undocked"
    sensor_overrides[SensorName.BATTERY_CHARGING] = True

    # Act and Assert
    with pytest.raises(RobotDockingStatusMismatchException):
        robot.initiate_return_home("00000000-0000-0000-0000-000000000000")


@pytest.fixture()
def no_sleep(mocker: MockerFixture) -> mock.MagicMock:
    return mocker.patch("isar_taurob.robotinterface.time")


@pytest.fixture()
def scheduled_actions(mocker: MockerFixture) -> mock.MagicMock:
    return mocker.patch.object(API, "get_scheduled_actions", return_value=[])


@pytest.fixture()
def set_robot_state(
    robot: Robot,
    mocked_sensors: mock.MagicMock,
    sensor_overrides: dict,
    scheduled_actions: mock.MagicMock,
    no_sleep: mock.MagicMock,
) -> Callable[..., None]:
    """Makes the robot online and returns a function updating the robot state."""
    robot.telemetry.last_update_from_robot = datetime.now(UTC)
    seconds_in_state: float = 0.0

    def _snapshot() -> dict[str, SensorValue]:
        snapshot = build_snapshot(**sensor_overrides)
        snapshot[SensorName.ROBOT_STATE].changed = datetime.now(UTC) - timedelta(
            seconds=seconds_in_state
        )
        return snapshot

    mocked_sensors.side_effect = _snapshot

    def _set_robot_state(state: str, seconds: float = 0.0) -> None:
        nonlocal seconds_in_state
        seconds_in_state = seconds
        sensor_overrides[SensorName.ROBOT_STATE] = state
        robot.telemetry.invalidate_sensor_cache()

    return _set_robot_state


@pytest.fixture()
def robot_dropped_from_dock(
    docked_robot: Robot, set_robot_state: Callable[..., None]
) -> Robot:
    """A robot that was docked and has just been reported as idle."""
    assert docked_robot.robot_status() == RobotStatus.Home
    set_robot_state("idle")
    return docked_robot


def test_robot_status_is_home_while_redocking(
    robot_dropped_from_dock: Robot, set_robot_state: Callable[..., None]
) -> None:
    robot: Robot = robot_dropped_from_dock
    statuses: list[RobotStatus] = []

    for state, seconds_in_state in [
        ("idle", settings.ROBOT_REDOCK_IDLE_GRACE_SECONDS - 5),
        ("redocking", 0),
        ("redocking", 3600),
        ("docked", 0),
    ]:
        set_robot_state(state, seconds_in_state)
        statuses.append(robot.robot_status())

    assert statuses == [RobotStatus.Home] * 4


def test_robot_status_is_not_home_when_idle_too_long_after_docked(
    robot_dropped_from_dock: Robot, set_robot_state: Callable[..., None]
) -> None:
    set_robot_state("idle", settings.ROBOT_REDOCK_IDLE_GRACE_SECONDS + 1)

    assert robot_dropped_from_dock.robot_status() == RobotStatus.Available


def test_robot_status_is_not_home_when_idle_without_having_been_docked(
    robot: Robot, set_robot_state: Callable[..., None]
) -> None:
    set_robot_state("idle")

    assert robot.robot_status() == RobotStatus.Available


def test_robot_status_is_not_home_when_idle_after_failed_redocking(
    robot_dropped_from_dock: Robot, set_robot_state: Callable[..., None]
) -> None:
    robot: Robot = robot_dropped_from_dock
    set_robot_state("redocking")
    assert robot.robot_status() == RobotStatus.Home

    set_robot_state("idle")

    assert robot.robot_status() == RobotStatus.Available


def test_initiate_mission_is_rejected_while_redocking(
    robot_dropped_from_dock: Robot,
) -> None:
    with pytest.raises(RobotInfeasibleMissionException, match="redocking"):
        robot_dropped_from_dock.initiate_mission(take_image_mission())


def test_initiate_return_home_is_disregarded_while_redocking(
    robot_dropped_from_dock: Robot, sensor_overrides: dict
) -> None:
    sensor_overrides[SensorName.LOCALIZATION_RUNNING] = False

    with pytest.raises(RobotAlreadyHomeException):
        robot_dropped_from_dock.initiate_return_home("mission_id")
