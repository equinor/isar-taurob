import datetime
import json
import logging
from json import JSONDecodeError
from logging import Logger

from robot_interface.models.exceptions.robot_exceptions import (
    RobotAPIException,
    RobotCommunicationException,
    RobotTelemetryNoUpdateException,
)
from robot_interface.models.robots.battery_state import BatteryState

from isar_taurob.config import settings
from isar_taurob.robot.api.api import API
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose


class TaurobTelemetry:
    def __init__(self, robot_api: API) -> None:
        self.robot_api: API = robot_api
        self.number_of_joints: int = settings.ROBOT_NUMBER_OF_JOINTS
        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)
        self.last_battery_measurement: float = 100.0
        self.last_pressure_measurement: float | None = None
        self.last_joint_temps: dict[str, float] = {}
        self.last_cpu_load: float | None = None

    def get_robot_pose(self) -> TaurobPose:
        pose_sensor: dict = self.robot_api.get_sensor_value("pose.base")

        self.robot_api.update_last_robot_update_time(pose_sensor)

        try:
            robot_pose: dict = json.loads(pose_sensor["value"])
            return TaurobPose(
                x=robot_pose["x"],
                y=robot_pose["y"],
                theta=robot_pose["theta"],
                map_name=robot_pose["floor"],
            )
        except JSONDecodeError, TypeError, KeyError:
            error_description: str = (
                f"Failed to decode robot pose from API: {pose_sensor}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def get_robot_battery(self) -> tuple[float, BatteryState]:
        battery_level_sensor: dict = self.robot_api.get_sensor_value(
            "health.battery_level"
        )
        last_updated_time = self.robot_api.update_last_robot_update_time(
            battery_level_sensor
        )
        time_now = datetime.datetime.now(datetime.UTC)
        if time_now - last_updated_time > datetime.timedelta(minutes=2):
            raise RobotTelemetryNoUpdateException(
                "The battery value is outdated. No battery message should be sent. "
            )

        battery_charging_sensor: dict = self.robot_api.get_sensor_value(
            "health.battery_charging"
        )
        self.robot_api.update_last_robot_update_time(battery_charging_sensor)

        try:
            battery_level_percentage: float = battery_level_sensor["value"]
            battery_charging: bool = battery_charging_sensor["value"]
            self.last_battery_measurement = battery_level_percentage
            return (
                battery_level_percentage,
                BatteryState.Charging if battery_charging else BatteryState.Normal,
            )

        except JSONDecodeError, TypeError, KeyError:
            error_description: str = (
                f"Failed to decode robot battery from API: Battery level {battery_level_sensor}, battery charging: {battery_charging_sensor}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def get_robot_obstacle_status(self) -> bool:
        obstacle_status_sensor: dict = {}
        try:
            obstacle_status_sensor = self.robot_api.get_sensor_value(
                "sensor.obstacle_status"
            )
            self.robot_api.update_last_robot_update_time(obstacle_status_sensor)

            obstacle_detected: bool = obstacle_status_sensor["value"] != "{}"
            return obstacle_detected

        except JSONDecodeError, TypeError, KeyError:
            error_description: str = (
                f"Failed to decode robot obstacle from API: {obstacle_status_sensor}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def get_robot_pressure_level(self) -> float:
        pressure_level_sensor: dict = self.robot_api.get_sensor_value(
            "sensor.pcu.pressure_1"
        )
        self.robot_api.update_last_robot_update_time(pressure_level_sensor)

        last_updated_time = self.robot_api.update_last_robot_update_time(
            pressure_level_sensor
        )
        time_now = datetime.datetime.now(datetime.UTC)
        if time_now - last_updated_time > datetime.timedelta(minutes=1):
            raise RobotTelemetryNoUpdateException(
                "The pressure value is outdated. No pressure message should be sent. "
            )

        try:
            pressure_level_percentage: float = pressure_level_sensor["value"]
            self.last_pressure_measurement = pressure_level_percentage
            return pressure_level_percentage

        except JSONDecodeError, TypeError, KeyError:
            error_description: str = (
                f"Failed to decode robot pressure "
                f"value from API: {pressure_level_sensor}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def update_cpu_load(self) -> None:
        try:
            cpu_load_sensor: dict = self.robot_api.get_sensor_value("health.cpuload")
            self.robot_api.update_last_robot_update_time(cpu_load_sensor)
            self.last_cpu_load = float(cpu_load_sensor["value"])

        except RobotAPIException, TypeError, KeyError, ValueError:
            error_description: str = (
                f"Failed to decode robot CPU load from API: {cpu_load_sensor}"
            )
            self.logger.warning(error_description)
        except RobotCommunicationException as e:
            self.logger.warning(
                f"Failed to communicate with robot API when updating CPU load: {e}"
            )

    def update_joint_temperatures(self) -> None:
        sensors: list[str] = [
            "health.uecu.base_r_j1.temperature",
            "health.uecu.base_l_flipper.temperature",
        ]

        for sensor_key in sensors:
            try:
                data: dict = self.robot_api.get_sensor_value(sensor_key)
                if "value" in data:
                    self.last_joint_temps[sensor_key] = float(data["value"])
                    self.robot_api.update_last_robot_update_time(data)
                else:
                    self.logger.warning(
                        f"Sensor {sensor_key} data missing 'value' key: {data}"
                    )

            except (RobotAPIException, ValueError, KeyError) as e:
                self.logger.warning(
                    f"Failed to update temperature for {sensor_key}: {e}"
                )
            except RobotCommunicationException as e:
                self.logger.warning(
                    f"Failed to communicate with robot API when updating temperature for {sensor_key}: {e}"
                )

    def get_mission_failure_reason(self) -> tuple[str, datetime.datetime]:
        mission_failure_sensor: dict = self.robot_api.get_sensor_value(
            "last_mission_failure_reason"
        )
        timestamp = self.robot_api.update_last_robot_update_time(mission_failure_sensor)

        try:
            mission_failure_reason_json: str = mission_failure_sensor["value"]
            mission_failure_reason_response: dict = json.loads(
                mission_failure_reason_json
            )
            failure_reason: str = mission_failure_reason_response["failure_reason"]
            return (failure_reason, timestamp)

        except JSONDecodeError, TypeError, KeyError:
            error_description: str = (
                f"Failed to decode mission failure reason from API: "
                f"{mission_failure_sensor}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def get_robot_remote_stop(self) -> bool:
        robot_remote_stop: dict = self.robot_api.get_sensor_value("robot_remote_stop")
        self.robot_api.update_last_robot_update_time(robot_remote_stop)

        try:
            remote_stop: bool = robot_remote_stop["value"]
            return remote_stop

        except JSONDecodeError, TypeError, KeyError:
            error_description: str = (
                f"Failed to decode robot remote stop value from API: "
                f"{robot_remote_stop}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)
