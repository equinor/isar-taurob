import logging
from functools import partial
from threading import Lock

from opentelemetry.metrics import CallbackOptions, Meter, Observation
from robot_interface.models.exceptions.robot_exceptions import RobotException

from isar_taurob.robot.telemetry.pose_snapshot import PoseSnapshot
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry

logger = logging.getLogger(__name__)

POSE_METRIC_FIELDS = {
    "x": ("m", "Robot base position X in the Taurob navigation map"),
    "y": ("m", "Robot base position Y in the Taurob navigation map"),
    "theta": ("rad", "Robot base heading in the Taurob navigation map"),
    "confidence": ("1", "Last reported localization confidence of the robot base pose"),
    "source_timestamp": ("s", "Taurob pose.base changed timestamp in Unix seconds"),
}


class PoseMetrics:
    def __init__(
        self, meter: Meter, telemetry: TaurobTelemetry, robot_name: str, isar_id: str
    ) -> None:
        self.telemetry = telemetry
        self.attributes = {"robot_name": robot_name, "isar_id": isar_id}
        self._snapshot: PoseSnapshot | None = None
        self._remaining_fields: set[str] = set()
        self._collection_lock = Lock()
        for field, (unit, description) in POSE_METRIC_FIELDS.items():
            meter.create_observable_gauge(
                name=f"robot.pose.{field}",
                callbacks=[partial(self.observe_snapshot_field, field)],
                unit=unit,
                description=description,
            )

    def observe_snapshot_field(
        self, field: str, _: CallbackOptions
    ) -> list[Observation]:
        with self._collection_lock:
            if field not in self._remaining_fields:
                self._start_snapshot_collection(field)
            else:
                self._remaining_fields.remove(field)

            if self._snapshot is None:
                return []
            value = getattr(self._snapshot, field)
            if value is None:
                return []
            return [Observation(value=value, attributes=self.attributes)]

    def _start_snapshot_collection(self, first_field: str) -> None:
        self._snapshot = None
        self._remaining_fields = set(POSE_METRIC_FIELDS) - {first_field}
        try:
            self._snapshot = self.telemetry.get_robot_pose_snapshot()
        except RobotException as e:
            logger.warning(f"Failed to read robot pose snapshot: {e.error_description}")
