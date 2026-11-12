import datetime
import functools
import logging
from enum import StrEnum
from logging import Logger
from typing import Any

from pydantic import BaseModel, ConfigDict, field_validator

from isar_taurob.config import settings

logger: Logger = logging.getLogger(settings.LOGGER_NAME)

TAUROB_TIME_FORMAT: str = "%Y-%m-%dT%H:%M:%S.%f%z %Z"


class SensorName(StrEnum):
    """Taurob API names of the sensors this integration reads."""

    POSE_BASE = "pose.base"
    BATTERY_LEVEL = "health.battery_level"
    BATTERY_CHARGING = "health.battery_charging"
    CPU_LOAD = "health.cpuload"
    TEMPERATURE_BASE_R_J1 = "health.uecu.base_r_j1.temperature"
    TEMPERATURE_BASE_L_FLIPPER = "health.uecu.base_l_flipper.temperature"
    OBSTACLE_STATUS = "sensor.obstacle_status"
    PRESSURE = "sensor.pcu.pressure_1"
    REMOTE_STOP = "robot_remote_stop"
    ROBOT_STATE = "robot.state"
    GAS_CONCENTRATION = "sensor.gas.xam8000"
    LOCALIZATION_RUNNING = "robot.localization_running"
    LAST_MISSION_FAILURE_REASON = "last_mission_failure_reason"


@functools.lru_cache(maxsize=256)
def _parse_taurob_time(value: str) -> datetime.datetime | None:
    # Cached so an unparsable timestamp is only logged once, not on every refresh
    try:
        return datetime.datetime.strptime(value, TAUROB_TIME_FORMAT).replace(
            tzinfo=datetime.UTC
        )
    except ValueError as e:
        logger.warning(
            f"Could not parse time string '{value}' from robot API: {e}. "
            f"Treating the sensor timestamp as unknown."
        )
        return None


class SensorValue(BaseModel):
    """A Taurob API sensor entry. ``value`` and ``changed`` are ``None`` for
    sensors the robot has never published."""

    model_config = ConfigDict(extra="ignore")

    name: str
    value: Any | None = None
    changed: datetime.datetime | None = None

    @field_validator("changed", mode="before")
    @classmethod
    def _parse_changed(cls, value: Any) -> datetime.datetime | None:
        if value is None or isinstance(value, datetime.datetime):
            return value
        if isinstance(value, str):
            return _parse_taurob_time(value)
        return None

    def is_older_than(self, max_age: datetime.timedelta) -> bool:
        """A value the robot has never published is considered stale."""
        if self.changed is None:
            return True
        return datetime.datetime.now(datetime.UTC) - self.changed > max_age
