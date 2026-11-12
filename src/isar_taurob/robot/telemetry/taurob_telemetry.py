import contextlib
import datetime
import json
import logging
import math
import threading
import time
from logging import Logger
from typing import Any

from robot_interface.models.exceptions.robot_exceptions import (
    RobotAPIException,
    RobotException,
    RobotTelemetryNoUpdateException,
)
from robot_interface.models.robots.battery_state import BatteryState

from isar_taurob.config import settings
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.sensor import SensorName, SensorValue
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose
from isar_taurob.robot.telemetry.pose_snapshot import PoseSnapshot

DECODE_ERRORS: tuple[type[Exception], ...] = (TypeError, KeyError, ValueError)

# Sensors whose value must not be published when the robot stopped updating it.
# Other sensors return their last value regardless of age.
SENSOR_MAX_AGE: dict[SensorName, datetime.timedelta] = {
    SensorName.BATTERY_LEVEL: datetime.timedelta(minutes=2),
    SensorName.PRESSURE: datetime.timedelta(minutes=1),
    SensorName.GAS_CONCENTRATION: datetime.timedelta(minutes=0.5),
}

JOINT_TEMPERATURE_SENSORS: tuple[SensorName, ...] = (
    SensorName.TEMPERATURE_BASE_R_J1,
    SensorName.TEMPERATURE_BASE_L_FLIPPER,
)


class TaurobTelemetry:
    """Reads robot sensors from a cached ``GET /sensor/all`` snapshot that is
    refreshed on read when older than ``SENSOR_REFRESH_INTERVAL``."""

    def __init__(self, robot_api: API) -> None:
        self.robot_api: API = robot_api
        self.number_of_joints: int = settings.ROBOT_NUMBER_OF_JOINTS
        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)
        self.last_battery_measurement: float = 100.0
        self.last_pressure_measurement: float | None = None
        self.last_update_from_robot: datetime.datetime = datetime.datetime.min.replace(
            tzinfo=datetime.UTC
        )

        self._sensor_cache: dict[str, SensorValue] = {}
        self._next_refresh: float = 0.0
        self._refresh_error: RobotException | None = None
        self._sensor_cache_lock: threading.RLock = threading.RLock()

    def invalidate_sensor_cache(self) -> None:
        """Force the next read to fetch a fresh snapshot."""
        with self._sensor_cache_lock:
            self._next_refresh = 0.0

    def refresh_sensor_cache(self) -> None:
        """Merge a fresh snapshot into the cache. ``GET /sensor/all`` does not
        always return the same sensors, so missing ones keep their last value.
        A failure is kept until the next refresh so concurrent readers don't
        all retry the request."""
        with self._sensor_cache_lock:
            self._next_refresh = time.monotonic() + settings.SENSOR_REFRESH_INTERVAL
            try:
                snapshot: dict[str, SensorValue] = (
                    self.robot_api.get_all_sensor_values()
                )
                if not snapshot:
                    raise RobotAPIException(
                        error_description=(
                            "The snapshot returned by the robot API contained no "
                            "usable sensor entries"
                        )
                    )
            except RobotException as e:
                self._refresh_error = e
                raise

            self._refresh_error = None
            self._sensor_cache.update(snapshot)
            for sensor in snapshot.values():
                self._record_robot_update(sensor)

    def _record_robot_update(self, sensor: SensorValue) -> datetime.datetime | None:
        if sensor.changed is not None:
            self.last_update_from_robot = max(
                self.last_update_from_robot, sensor.changed
            )
        return sensor.changed

    def _get_sensor(self, sensor_name: SensorName) -> SensorValue:
        with self._sensor_cache_lock:
            if time.monotonic() >= self._next_refresh:
                # The failure is stored in _refresh_error and raised below
                with contextlib.suppress(RobotException):
                    self.refresh_sensor_cache()
            if self._refresh_error is not None:
                raise self._refresh_error
            sensor: SensorValue | None = self._sensor_cache.get(sensor_name)

        if sensor is None:
            error_description: str = (
                f"Sensor '{sensor_name}' was not present in the snapshot returned "
                f"by the robot API"
            )
            self.logger.debug(error_description)
            raise RobotAPIException(error_description=error_description)

        max_age: datetime.timedelta | None = SENSOR_MAX_AGE.get(sensor_name)
        if max_age is not None and sensor.is_older_than(max_age):
            raise RobotTelemetryNoUpdateException(
                f"The '{sensor_name}' value is outdated. "
                f"No telemetry message should be sent."
            )

        return sensor

    def _get_value(self, sensor_name: SensorName) -> Any:
        return self._get_sensor(sensor_name).value

    def _get_float(self, sensor_name: SensorName) -> float:
        value: Any = self._get_value(sensor_name)
        try:
            return float(value)
        except DECODE_ERRORS:
            error_description: str = (
                f"Failed to decode '{sensor_name}' as a number from API: {value!r}"
            )
            self.logger.warning(error_description)
            raise RobotAPIException(error_description=error_description)

    def get_robot_pose(self) -> TaurobPose:
        pose_value: Any = self._get_value(SensorName.POSE_BASE)

        try:
            robot_pose: dict = json.loads(pose_value)
            return TaurobPose(
                x=robot_pose["x"],
                y=robot_pose["y"],
                theta=robot_pose["theta"],
                map_name=robot_pose["floor"],
            )
        except DECODE_ERRORS:
            error_description: str = (
                f"Failed to decode robot pose from API: {pose_value}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def get_robot_pose_snapshot(self) -> PoseSnapshot:
        sensor = self._get_sensor(SensorName.POSE_BASE)
        try:
            snapshot = PoseSnapshot.model_validate_json(sensor.value)
        except (TypeError, ValueError) as e:
            raise RobotAPIException(
                error_description=f"Failed to decode pose.base snapshot: {e}"
            ) from e
        return snapshot.model_copy(update={"changed": sensor.changed})

    def get_robot_pose_confidence(self) -> float | None:
        """Return the last reported localization confidence from cached pose.base."""
        pose_value: Any = self._get_value(SensorName.POSE_BASE)
        try:
            pose = json.loads(pose_value)
            confidence = pose.get("confidence")
            if confidence is None:
                return None
            if isinstance(confidence, bool):
                raise TypeError("confidence must be numeric")
            value = float(confidence)
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError("confidence must be between 0 and 1")
            return value
        except (AttributeError, TypeError, ValueError) as e:
            raise RobotAPIException(
                error_description=f"Failed to decode pose.base confidence: {e}"
            ) from e

    def get_robot_battery(self) -> tuple[float, BatteryState]:
        battery_level_percentage: float = self._get_float(SensorName.BATTERY_LEVEL)
        battery_charging: bool = bool(self._get_value(SensorName.BATTERY_CHARGING))
        self.last_battery_measurement = battery_level_percentage
        return (
            battery_level_percentage,
            BatteryState.Charging if battery_charging else BatteryState.Normal,
        )

    def get_robot_obstacle_status(self) -> bool:
        return self._get_value(SensorName.OBSTACLE_STATUS) != "{}"

    def get_robot_is_localization_running(self) -> bool:
        return bool(self._get_value(SensorName.LOCALIZATION_RUNNING))

    def get_robot_pressure_level(self) -> float:
        pressure_level: float = self._get_float(SensorName.PRESSURE)
        self.last_pressure_measurement = pressure_level
        return pressure_level

    def get_robot_state(self) -> str:
        return self._get_value(SensorName.ROBOT_STATE)

    def get_robot_remote_stop(self) -> bool:
        return bool(self._get_value(SensorName.REMOTE_STOP))

    def get_cpu_load(self) -> float:
        return self._get_float(SensorName.CPU_LOAD)

    def get_joint_temperatures(self) -> dict[str, float]:
        """Temperatures of the joints that currently have a readable value."""
        temperatures: dict[str, float] = {}
        for sensor_name in JOINT_TEMPERATURE_SENSORS:
            try:
                temperatures[sensor_name] = self._get_float(sensor_name)
            except RobotAPIException:
                continue
        return temperatures

    def get_mission_failure_reason(self) -> tuple[str, datetime.datetime | None]:
        # Read directly, bypassing the cache, to get the just-finished mission
        mission_failure_sensor: SensorValue = self.robot_api.get_sensor_value(
            SensorName.LAST_MISSION_FAILURE_REASON
        )
        timestamp = self._record_robot_update(mission_failure_sensor)

        try:
            mission_failure_reason_response: dict = json.loads(
                mission_failure_sensor.value
            )
            failure_reason: str = mission_failure_reason_response["failure_reason"]
            return (failure_reason, timestamp)

        except DECODE_ERRORS:
            error_description: str = (
                f"Failed to decode mission failure reason from API: "
                f"{mission_failure_sensor}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)
