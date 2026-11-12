import os
from pathlib import Path
from typing import Any
from unittest import mock

import pytest
from pytest_mock import MockerFixture
from robot_interface.models.mission.task import (
    AcousticDetectionType,
    TakeAcousticMeasurement,
)

from isar_taurob import Robot
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.request_handler import RequestHandler
from isar_taurob.robot.api.sensor import SensorValue
from isar_taurob.robot.map import MapHandler
from isar_taurob.robot.scheduler.scheduler import Scheduler
from isar_taurob.robot.storage.storage import Storage
from isar_taurob.robot.taurob_model.arm import TaurobArm
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry
from tests.utilities.mock_api_return_values.mock_taurob_api import MockTaurobAPI
from tests.utilities.mock_models.mock_robot_variables import mock_pose, mock_target


def build_snapshot(
    changed: str | None = None,
    omit: list[str] | None = None,
    **overrides: Any,
) -> dict[str, SensorValue]:
    """A parsed ``GET /sensor/all`` snapshot, as TaurobTelemetry caches it."""
    snapshot: dict[str, SensorValue] = {
        entry["name"]: SensorValue.model_validate(entry)
        for entry in MockTaurobAPI.get_all_sensor_values(changed=changed, omit=omit)
    }
    for name, value in overrides.items():
        snapshot[name].value = value
    return snapshot


@pytest.fixture()
def sensor_overrides() -> dict[str, Any]:
    """Sensor values to override in the snapshot served by ``mocked_sensors``.

    Request this fixture and assign into it before acting, e.g.
    ``sensor_overrides["robot.state"] = "docked"``.
    """
    return {}


@pytest.fixture()
def mocked_sensors(
    mocker: MockerFixture, sensor_overrides: dict[str, Any]
) -> mock.MagicMock:
    """Serve every telemetry read from a mock snapshot instead of the robot.

    The snapshot is rebuilt per call so timestamps stay fresh and late
    assignments to ``sensor_overrides`` are picked up.
    """
    return mocker.patch.object(
        API,
        "get_all_sensor_values",
        side_effect=lambda: build_snapshot(**sensor_overrides),
    )


@pytest.fixture()
def acoustic_task() -> TakeAcousticMeasurement:
    return TakeAcousticMeasurement(
        id="acoustic-task",
        robot_pose=mock_pose(),
        target=mock_target(),
        tag_id="test-tag",
        inspection_description="Leak detection",
        frequency_from=35000,
        frequency_to=40000,
        snr_value_threshold=70,
        detection_type=AcousticDetectionType.leak,
    )


@pytest.fixture()
def robot(map_handler, scheduler) -> Robot:
    robot: Robot = Robot(
        robot_name="william", isar_id="00000000-0000-0000-0000-000000000000"
    )
    robot.map_handler = map_handler
    robot.scheduler = scheduler
    return robot


@pytest.fixture()
def taurob_robot_api(request_handler) -> API:
    return API(request_handler)


@pytest.fixture()
def telemetry_service(taurob_robot_api) -> TaurobTelemetry:
    return TaurobTelemetry(robot_api=taurob_robot_api)


@pytest.fixture()
def request_handler() -> RequestHandler:
    return RequestHandler()


@pytest.fixture()
def map_handler() -> MapHandler:
    return MapHandler(asset_short_name="default")


@pytest.fixture()
def map_handler_loaded_maps() -> list[MapHandler]:
    here: Path = Path(__file__).parent.resolve()
    map_config_folder_path: Path = Path(
        here.parent.joinpath("src/isar_taurob/config/maps/")
    )
    not_checked: list[str] = ["not_in_use", "default"]
    map_handlers: list[MapHandler] = []
    for folder in os.listdir(map_config_folder_path):
        if (
            os.path.isdir(os.path.join(map_config_folder_path, folder))
            and folder in not_checked
        ):
            continue
        map_handlers.append(MapHandler(asset_short_name=folder))

    return map_handlers


@pytest.fixture()
def scheduler(taurob_robot_api, telemetry_service, map_handler) -> Scheduler:
    return Scheduler(
        robot_api=taurob_robot_api, telemetry=telemetry_service, map_handler=map_handler
    )


@pytest.fixture()
def storage(taurob_robot_api, telemetry_service) -> Storage:
    return Storage(robot_api=taurob_robot_api, robot_telemetry=telemetry_service)


@pytest.fixture()
def taurob_arm_4_joints() -> TaurobArm:
    return TaurobArm(number_of_joints=4)


@pytest.fixture()
def taurob_arm_2_joints() -> TaurobArm:
    return TaurobArm(number_of_joints=2)
