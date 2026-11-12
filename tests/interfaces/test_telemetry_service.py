import datetime

import pytest
from robot_interface.models.exceptions.robot_exceptions import (
    RobotAPIException,
    RobotCommunicationException,
    RobotTelemetryNoUpdateException,
)
from robot_interface.models.robots.battery_state import BatteryState

from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.sensor import SensorName, SensorValue
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose
from tests.conftest import build_snapshot as _snapshot
from tests.utilities.mock_api_return_values.mock_taurob_api import format_changed


def _stale_timestamp(minutes: int) -> str:
    stale = datetime.datetime.now(datetime.UTC) - datetime.timedelta(minutes=minutes)
    return format_changed(stale)


def _prime(telemetry_service, mocker, changed: str | None = None):
    get_all = mocker.patch.object(
        API, "get_all_sensor_values", return_value=_snapshot(changed)
    )
    telemetry_service.refresh_sensor_cache()
    return get_all


def test_get_robot_pose(telemetry_service, mocker):
    _prime(telemetry_service, mocker)

    assert telemetry_service.get_robot_pose() == TaurobPose(
        x=0.01, y=0.025, theta=6.283, map_name="test_map"
    )


def test_get_robot_pose_raises_on_undecodable_value(telemetry_service, mocker):
    snapshot = _snapshot()
    snapshot[SensorName.POSE_BASE] = SensorValue(
        name=SensorName.POSE_BASE,
        value='["this is not a dict"]',
        changed=datetime.datetime.now(datetime.UTC),
    )
    mocker.patch.object(API, "get_all_sensor_values", return_value=snapshot)
    telemetry_service.refresh_sensor_cache()

    with pytest.raises(RobotAPIException):
        telemetry_service.get_robot_pose()


def test_getters_read_from_the_snapshot(telemetry_service, mocker):
    get_all = mocker.patch.object(
        API, "get_all_sensor_values", return_value=_snapshot()
    )
    get_sensor = mocker.patch.object(API, "get_sensor_value")

    telemetry_service.refresh_sensor_cache()
    telemetry_service.get_robot_pose()
    telemetry_service.get_robot_battery()
    telemetry_service.get_robot_pressure_level()
    telemetry_service.get_robot_obstacle_status()
    telemetry_service.get_robot_remote_stop()
    telemetry_service.get_robot_state()
    telemetry_service.get_cpu_load()
    telemetry_service.get_joint_temperatures()

    assert get_all.call_count == 1
    get_sensor.assert_not_called()


def test_first_read_fetches_the_snapshot(telemetry_service, mocker):
    get_all = mocker.patch.object(
        API, "get_all_sensor_values", return_value=_snapshot()
    )

    assert telemetry_service.get_robot_state() == "idle"
    assert get_all.call_count == 1


def test_expired_snapshot_is_refreshed_on_read(telemetry_service, mocker):
    get_all = _prime(telemetry_service, mocker)

    telemetry_service._next_refresh = 0.0
    telemetry_service.get_robot_state()

    assert get_all.call_count == 2


def test_invalidate_forces_a_fresh_read(telemetry_service, mocker):
    get_all = _prime(telemetry_service, mocker)

    telemetry_service.get_robot_state()
    telemetry_service.invalidate_sensor_cache()
    telemetry_service.get_robot_state()

    assert get_all.call_count == 2


def test_get_robot_battery(telemetry_service, mocker):
    _prime(telemetry_service, mocker)

    assert telemetry_service.get_robot_battery() == (10, BatteryState.Normal)


def test_get_robot_state(telemetry_service, mocker):
    _prime(telemetry_service, mocker)

    assert telemetry_service.get_robot_state() == "idle"


def test_get_cpu_load_and_joint_temperatures(telemetry_service, mocker):
    _prime(telemetry_service, mocker)

    assert telemetry_service.get_cpu_load() == 12.5
    assert (
        telemetry_service.get_joint_temperatures()[SensorName.TEMPERATURE_BASE_R_J1]
        == 32
    )


def test_unreachable_api_raises_communication_exception(telemetry_service, mocker):
    # Must be a comms error so ISAR retries instead of failing the task
    get_all = mocker.patch.object(
        API,
        "get_all_sensor_values",
        side_effect=RobotCommunicationException(error_description="Unreachable"),
    )

    for _ in range(3):
        with pytest.raises(RobotCommunicationException):
            telemetry_service.get_robot_pose()

    # Concurrent and repeated readers share one failed attempt per interval
    assert get_all.call_count == 1


def test_failed_refresh_recovers_on_next_refresh(telemetry_service, mocker):
    mocker.patch.object(
        API,
        "get_all_sensor_values",
        side_effect=RobotCommunicationException(error_description="Unreachable"),
    )
    with pytest.raises(RobotCommunicationException):
        telemetry_service.get_robot_state()

    _prime(telemetry_service, mocker)

    assert telemetry_service.get_robot_state() == "idle"


def test_empty_snapshot_does_not_clear_the_cache(telemetry_service, mocker):
    _prime(telemetry_service, mocker)

    mocker.patch.object(API, "get_all_sensor_values", return_value={})
    with pytest.raises(RobotAPIException):
        telemetry_service.refresh_sensor_cache()
    with pytest.raises(RobotAPIException):
        telemetry_service.get_robot_state()

    partial = _snapshot(omit=[SensorName.ROBOT_STATE])
    mocker.patch.object(API, "get_all_sensor_values", return_value=partial)
    telemetry_service.refresh_sensor_cache()

    assert telemetry_service.get_robot_state() == "idle"


def test_non_numeric_value_raises_api_exception(telemetry_service, mocker):
    snapshot = _snapshot()
    snapshot[SensorName.CPU_LOAD] = SensorValue(
        name=SensorName.CPU_LOAD,
        value="not a number",
        changed=datetime.datetime.now(datetime.UTC),
    )
    mocker.patch.object(API, "get_all_sensor_values", return_value=snapshot)
    telemetry_service.refresh_sensor_cache()

    with pytest.raises(RobotAPIException):
        telemetry_service.get_cpu_load()


def test_partial_snapshot_keeps_previously_known_sensors(telemetry_service, mocker):
    _prime(telemetry_service, mocker)

    partial = _snapshot(omit=[SensorName.ROBOT_STATE, SensorName.PRESSURE])
    mocker.patch.object(API, "get_all_sensor_values", return_value=partial)

    telemetry_service.refresh_sensor_cache()

    assert telemetry_service.get_robot_state() == "idle"
    assert telemetry_service.get_robot_pressure_level() == 0.95


def test_snapshot_from_production_is_usable(telemetry_service, mocker):
    """Guards against drift between SensorName and the names the robot uses."""
    snapshot = _snapshot()
    for sensor_name in SensorName:
        assert sensor_name in snapshot, f"{sensor_name} is missing from the snapshot"
        assert (
            snapshot[sensor_name].changed is not None
        ), f"{sensor_name} has an unparsable timestamp"


def test_missing_sensor_raises(telemetry_service, mocker):
    _prime(telemetry_service, mocker)
    del telemetry_service._sensor_cache[SensorName.PRESSURE]

    with pytest.raises(RobotAPIException):
        telemetry_service.get_robot_pressure_level()


@pytest.mark.parametrize(
    "getter",
    ["get_robot_battery", "get_robot_pressure_level"],
)
def test_outdated_value_raises_for_battery_and_pressure(
    telemetry_service, mocker, getter
):
    _prime(telemetry_service, mocker, changed=_stale_timestamp(minutes=10))

    with pytest.raises(RobotTelemetryNoUpdateException):
        getattr(telemetry_service, getter)()


def test_outdated_pose_is_returned_without_raising(telemetry_service, mocker):
    """Pose is legitimately stale while docked."""
    _prime(telemetry_service, mocker, changed=_stale_timestamp(minutes=10))

    assert telemetry_service.get_robot_pose() == TaurobPose(
        x=0.01, y=0.025, theta=6.283, map_name="test_map"
    )


def test_outdated_pose_confidence_is_still_reported(telemetry_service, mocker):
    snapshot = _snapshot(changed=_stale_timestamp(minutes=60 * 24))
    snapshot[SensorName.POSE_BASE].value = '{"confidence": 0.46}'
    mocker.patch.object(API, "get_all_sensor_values", return_value=snapshot)
    assert telemetry_service.get_robot_pose_confidence() == 0.46


def test_offline_robot_does_not_look_like_a_communication_failure(
    telemetry_service, mocker
):
    """Old sensor values must not be reported as a communication failure."""
    _prime(telemetry_service, mocker, changed=_stale_timestamp(minutes=60 * 24 * 14))

    assert telemetry_service.get_robot_pose() is not None

    with pytest.raises(RobotTelemetryNoUpdateException):
        telemetry_service.get_robot_battery()


def test_refresh_records_last_robot_update_time(telemetry_service, mocker):
    snapshot = _snapshot()
    mocker.patch.object(API, "get_all_sensor_values", return_value=snapshot)

    telemetry_service.refresh_sensor_cache()

    newest = max(
        sensor.changed for sensor in snapshot.values() if sensor.changed is not None
    )
    assert telemetry_service.last_update_from_robot == newest
