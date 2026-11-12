import datetime
import json
import time

import pytest
from opentelemetry.sdk.metrics import MeterProvider
from opentelemetry.sdk.metrics.export import InMemoryMetricReader
from robot_interface.models.exceptions.robot_exceptions import (
    RobotCommunicationException,
)

from isar_taurob.open_telemetry.metrics_handler import MetricsHandler
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.sensor import SensorName
from tests.conftest import build_snapshot


@pytest.fixture()
def metric_reader(telemetry_service, mocker):
    reader = InMemoryMetricReader()
    provider = MeterProvider(metric_readers=[reader])
    mocker.patch(
        "isar_taurob.open_telemetry.metrics_handler.metrics.get_meter",
        return_value=provider.get_meter("isar.taurob"),
    )
    MetricsHandler(telemetry_service, "Evan", "isar-id")
    yield reader
    provider.shutdown()


def collect_points(reader):
    data = reader.get_metrics_data()
    return {
        metric.name: metric.data.data_points[0]
        for resource in data.resource_metrics
        for scope in resource.scope_metrics
        for metric in scope.metrics
    }


@pytest.fixture()
def pose_sensor(mocker):
    snapshot = build_snapshot()
    sensor = snapshot[SensorName.POSE_BASE]
    sensor.value = json.dumps({"x": 10, "y": 12.5, "theta": 0.3, "confidence": 0.46})
    sensor.changed = datetime.datetime(2026, 11, 10, 12, 0, 0, 123456, datetime.UTC)
    mocker.patch.object(API, "get_all_sensor_values", return_value=snapshot)
    return sensor


def test_pose_metrics_export_one_snapshot_with_source_timestamp(
    telemetry_service, metric_reader, pose_sensor, mocker
):
    read_snapshot = mocker.spy(telemetry_service, "get_robot_pose_snapshot")
    expected = {
        "robot.pose.x": 10,
        "robot.pose.y": 12.5,
        "robot.pose.theta": 0.3,
        "robot.pose.confidence": 0.46,
        "robot.pose.source_timestamp": pose_sensor.changed.timestamp(),
    }

    for _ in range(2):
        collection_start = time.time_ns()
        points = collect_points(metric_reader)
        collection_end = time.time_ns()
        assert {name: points[name].value for name in expected} == expected
        assert all(
            points[name].attributes == {"robot_name": "Evan", "isar_id": "isar-id"}
            for name in expected
        )
        assert (
            collection_start <= points["robot.pose.x"].time_unix_nano <= collection_end
        )
        assert len({points[name].time_unix_nano for name in expected}) == 1

    assert read_snapshot.call_count == 2


def test_pose_fields_stay_consistent_when_sensor_changes_during_collection(
    telemetry_service, metric_reader, pose_sensor, mocker
):
    read_snapshot = telemetry_service.get_robot_pose_snapshot
    original_timestamp = pose_sensor.changed.timestamp()

    def read_then_update_sensor():
        snapshot = read_snapshot()
        pose_sensor.value = '{"x": 20, "y": 25, "theta": 0.6, "confidence": 0.9}'
        pose_sensor.changed += datetime.timedelta(seconds=3)
        telemetry_service.invalidate_sensor_cache()
        return snapshot

    mocker.patch.object(
        telemetry_service,
        "get_robot_pose_snapshot",
        side_effect=read_then_update_sensor,
    )

    points = collect_points(metric_reader)

    assert points["robot.pose.x"].value == 10
    assert points["robot.pose.y"].value == 12.5
    assert points["robot.pose.theta"].value == 0.3
    assert points["robot.pose.confidence"].value == 0.46
    assert points["robot.pose.source_timestamp"].value == original_timestamp


def test_next_collection_records_updated_pose_and_timestamp(
    telemetry_service, metric_reader, pose_sensor
):
    collect_points(metric_reader)
    pose_sensor.value = '{"x": 20, "y": 25, "theta": 0.6, "confidence": 0.9}'
    pose_sensor.changed += datetime.timedelta(seconds=3)
    telemetry_service.invalidate_sensor_cache()

    points = collect_points(metric_reader)

    assert points["robot.pose.x"].value == 20
    assert points["robot.pose.y"].value == 25
    assert points["robot.pose.theta"].value == 0.6
    assert points["robot.pose.confidence"].value == 0.9
    assert (
        points["robot.pose.source_timestamp"].value == pose_sensor.changed.timestamp()
    )


def test_unknown_source_timestamp_is_not_replaced_with_collection_time(
    metric_reader, pose_sensor
):
    collect_points(metric_reader)
    pose_sensor.changed = None

    points = collect_points(metric_reader)

    assert points["robot.pose.x"].value == 10
    assert "robot.pose.source_timestamp" not in points


@pytest.mark.parametrize("confidence", [None, "missing"])
def test_pose_is_exported_without_unavailable_confidence(
    metric_reader, pose_sensor, confidence
):
    pose = json.loads(pose_sensor.value)
    if confidence == "missing":
        del pose["confidence"]
    else:
        pose["confidence"] = confidence
    pose_sensor.value = json.dumps(pose)

    points = collect_points(metric_reader)

    assert points["robot.pose.x"].value == 10
    assert (
        points["robot.pose.source_timestamp"].value == pose_sensor.changed.timestamp()
    )
    assert "robot.pose.confidence" not in points


@pytest.mark.parametrize(
    "pose_value",
    [
        "not json",
        "null",
        '{"x": 10}',
        '{"x": true, "y": 12.5, "theta": 0}',
        '{"x": 1e999, "y": 12.5, "theta": 0}',
        '{"x": 10, "y": 12.5, "theta": 0, "confidence": true}',
        '{"x": 10, "y": 12.5, "theta": 0, "confidence": 2}',
    ],
)
def test_invalid_snapshot_does_not_export_misleading_pose_metrics(
    metric_reader, pose_sensor, pose_value
):
    collect_points(metric_reader)
    pose_sensor.value = pose_value

    points = collect_points(metric_reader)

    assert not any(name.startswith("robot.pose.") for name in points)
    assert "robot.cpu.load" in points


@pytest.mark.parametrize(
    "error",
    [
        RobotCommunicationException(error_description="unreachable"),
        RuntimeError("broken"),
    ],
)
def test_failed_pose_read_does_not_reexport_old_pose_or_interrupt_other_metrics(
    telemetry_service, metric_reader, pose_sensor, mocker, error
):
    collect_points(metric_reader)
    mocker.patch.object(telemetry_service, "get_robot_pose_snapshot", side_effect=error)

    for _ in range(2):
        points = collect_points(metric_reader)

        assert not any(name.startswith("robot.pose.") for name in points)
        assert points["robot.cpu.load"].value == 12.5
