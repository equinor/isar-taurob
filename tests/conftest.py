import os
from pathlib import Path

import pytest

from isar_taurob import Robot
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.request_handler import RequestHandler
from isar_taurob.robot.map import MapHandler
from isar_taurob.robot.scheduler.scheduler import Scheduler
from isar_taurob.robot.storage.storage import Storage
from isar_taurob.robot.taurob_model.arm import TaurobArm
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry


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
def storage(taurob_robot_api) -> Storage:
    return Storage(robot_api=taurob_robot_api)


@pytest.fixture()
def taurob_arm_4_joints() -> TaurobArm:
    return TaurobArm(number_of_joints=4)


@pytest.fixture()
def taurob_arm_2_joints() -> TaurobArm:
    return TaurobArm(number_of_joints=2)
