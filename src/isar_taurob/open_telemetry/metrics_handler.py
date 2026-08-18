import logging

from opentelemetry import metrics
from opentelemetry.metrics import CallbackOptions, Meter, Observation

from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry

logger = logging.getLogger(__name__)


class MetricsHandler:
    def __init__(
        self, telemetry: TaurobTelemetry, robot_name: str, isar_id: str
    ) -> None:
        self.telemetry = telemetry
        self.robot_name = robot_name
        self.isar_id = isar_id

        self.meter: Meter = metrics.get_meter("isar.taurob")
        self._setup_observable_gauges()

    def _setup_observable_gauges(self) -> None:
        self.meter.create_observable_gauge(
            name="robot.battery.level",
            callbacks=[self._observe_battery_level],
            unit="%",
            description="Current battery level of the robot",
        )
        self.meter.create_observable_gauge(
            name="robot.pressure.level",
            callbacks=[self._observe_pressure_level],
            unit="bar",
            description="Current pressure level of the robot",
        )
        self.meter.create_observable_gauge(
            name="robot.temperature.joint",
            callbacks=[self._observe_joint_temperatures],
            unit="Celsius",
            description="Temperature of robot joints and flippers",
        )
        self.meter.create_observable_gauge(
            name="robot.cpu.load",
            callbacks=[self._observe_cpu_load],
            unit="%",
            description="Current CPU load of the robot",
        )

    def _observe_battery_level(self, _: CallbackOptions) -> list[Observation]:
        return [
            Observation(
                value=self.telemetry.last_battery_measurement,
                attributes={"robot_name": self.robot_name, "isar_id": self.isar_id},
            )
        ]

    def _observe_pressure_level(self, _: CallbackOptions) -> list[Observation]:
        last_pressure_measurement: float | None = (
            self.telemetry.last_pressure_measurement
        )
        if last_pressure_measurement is None:
            return []

        return [
            Observation(
                value=last_pressure_measurement,
                attributes={"robot_name": self.robot_name, "isar_id": self.isar_id},
            )
        ]

    def _observe_joint_temperatures(self, _: CallbackOptions) -> list[Observation]:
        observations: list[Observation] = []

        for sensor_key, temp_value in self.telemetry.last_joint_temps.items():
            observations.append(
                Observation(
                    value=temp_value,
                    attributes={
                        "robot_name": self.robot_name,
                        "isar_id": self.isar_id,
                        "taurob.sensor_key": sensor_key,
                    },
                )
            )
        return observations

    def _observe_cpu_load(self, _: CallbackOptions) -> list[Observation]:
        last_cpu_load: float | None = self.telemetry.last_cpu_load
        if last_cpu_load is None:
            return []

        return [
            Observation(
                value=last_cpu_load,
                attributes={"robot_name": self.robot_name, "isar_id": self.isar_id},
            )
        ]
