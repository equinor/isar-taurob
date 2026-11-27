import asyncio
import threading
from datetime import UTC, datetime, timedelta
from unittest.mock import AsyncMock, Mock

import pytest
from robot_interface.recording_models import SegmentTiming

from isar_taurob.robot.api.api import API
from isar_taurob.robot.recording.mission_recorder import TaurobMissionRecorder
from isar_taurob.robot.recording.models import (
    CaptureMetadata,
    DriverCameraRecording,
    DriverRecordingSession,
    RecordedSegment,
)
from isar_taurob.robot.recording.pose_collector import POSE_LOG_FILENAME
from isar_taurob.robot.recording.session_finalizer import SessionFinalizer
from tests.conftest import build_snapshot


@pytest.fixture
def recorder(tmp_path):
    recorder = TaurobMissionRecorder.__new__(TaurobMissionRecorder)
    recorder.directory = tmp_path
    recorder.workers = {}
    recorder.publisher = Mock()
    recorder._finalize = AsyncMock()
    recorder._publish = AsyncMock()
    return recorder


def test_start_persists_mission_name(recorder):
    recorder.closing = False
    recorder.sessions = {}
    recorder.capture = AsyncMock(side_effect=lambda stop, *_: stop.set())

    async def start():
        recorder._start_session("mission", "Perimeter")
        await recorder.workers["mission"][1]

    asyncio.run(start())

    session = recorder.sessions["mission"]
    recovered = DriverRecordingSession.model_validate_json(
        (recorder._session_directory(session) / "session.json").read_text()
    )
    assert recovered.mission_name == "Perimeter"


def test_capture_persists_recoverable_catalog(recorder):
    session = DriverRecordingSession(run_id="mission", status="Starting", cameras={})

    async def capture(stop, open_segment, update_camera):
        assert (
            open_segment("front", CaptureMetadata(source_codec="H264")).suffix == ".mkv"
        )
        update_camera("front", CaptureMetadata(last_frame_at=123.5))
        stop.set()

    recorder.capture = capture
    asyncio.run(recorder._capture_session(session, asyncio.Event()))

    recovered = DriverRecordingSession.model_validate_json(
        (recorder._session_directory(session) / "session.json").read_text()
    )
    assert recovered == session
    assert recovered.status == "Processing"
    assert recovered.ended_at is not None
    assert recovered.cameras["front"].capture_metadata.source_codec == "H264"
    assert recovered.cameras["front"].capture_metadata.last_frame_at == 123.5
    assert recovered.cameras["front"].segments == 1
    recorder._finalize.assert_awaited_once_with(session)


@pytest.fixture
def finalization(tmp_path, mocker):
    def write_video(source, output, **options):
        output.write_bytes(b"video")

    for operation in ("convert_segment", "concatenate_segments"):
        mocker.patch(
            f"isar_taurob.robot.recording.session_finalizer.{operation}",
            new=AsyncMock(side_effect=write_video),
        )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.validate_video", new=AsyncMock()
    )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.probe_video_duration",
        new=AsyncMock(return_value=10.0),
    )
    (tmp_path / "session.json").write_text("{}")
    (tmp_path / "front-00001.mkv").write_bytes(b"footage")
    session = DriverRecordingSession(
        run_id="mission",
        status="Processing",
        cameras={
            "front": DriverCameraRecording(
                camera_id="front",
                key="front",
                status="Processing",
                capture_metadata=CaptureMetadata(source_codec="H264"),
            )
        },
    )
    publisher = Mock(publish_video=AsyncMock(return_value="opaque-reference"))
    return SessionFinalizer(publisher, Mock(), AsyncMock()), session, tmp_path


def test_success_removes_footage_before_publication(finalization):
    finalizer, session, directory = finalization

    async def publish(document):
        assert document.status == "Ready"
        assert not (directory / "front-00001.mkv").exists()

    finalizer.publish.side_effect = publish
    asyncio.run(finalizer.finalize_session(session, directory))

    assert {path.name for path in directory.iterdir()} == {"session.json", "finalized"}
    assert session.cameras["front"].artifact_reference == "opaque-reference"
    finalizer.publisher.publish_video.assert_awaited_once()


def test_failed_upload_expires_only_failed_camera_and_removes_all_footage(finalization):
    finalizer, session, directory = finalization
    session.cameras["arm"] = DriverCameraRecording(
        camera_id="arm",
        key="arm",
        status="Processing",
        capture_metadata=CaptureMetadata(rotation_degrees=90),
    )
    (directory / "arm-00001.mkv").write_bytes(b"footage")
    finalizer.publisher.publish_video.side_effect = [OSError(), "arm-ref", "front-ref"]

    asyncio.run(finalizer.finalize_session(session, directory))
    assert session.status == "Expired"
    assert session.cameras["front"].status == "Expired"
    assert "upload" in session.cameras["front"].error
    assert session.cameras["arm"].status == "Ready"
    assert {path.name for path in directory.iterdir()} == {"session.json", "finalized"}

    asyncio.run(finalizer.finalize_session(session, directory))
    assert session.status == "Expired"
    assert finalizer.publisher.publish_video.await_count == 2
    assert {path.name for path in directory.iterdir()} == {"session.json", "finalized"}


@pytest.mark.parametrize("upload_fails", [False, True])
def test_completed_session_is_not_republished_after_restart(finalization, upload_fails):
    finalizer, session, directory = finalization

    def persist(document):
        (directory / "session.json").write_text(document.model_dump_json())

    finalizer.persist = persist
    if upload_fails:
        finalizer.publisher.publish_video.side_effect = OSError()

    asyncio.run(finalizer.finalize_session(session, directory))
    asyncio.run(finalizer.finalize_session(session, directory))

    recovered = DriverRecordingSession.model_validate_json(
        (directory / "session.json").read_text()
    )
    restarted = SessionFinalizer(finalizer.publisher, persist, finalizer.publish)
    asyncio.run(restarted.finalize_session(recovered, directory))

    finalizer.publish.assert_awaited_once()
    finalizer.publisher.publish_video.assert_awaited_once()


@pytest.mark.parametrize("failure_stage", ["publication", "cleanup"])
@pytest.mark.parametrize("upload_fails", [False, True])
def test_maintenance_retries_publication_and_cleanup_after_restart(
    finalization, recorder, mocker, failure_stage, upload_fails
):
    finalizer, session, directory = finalization
    directory = recorder._session_directory(session)
    directory.mkdir()
    (directory / "front-00001.mkv").write_bytes(b"footage")
    session.ended_at = datetime.now(UTC).isoformat()
    finalizer.persist = recorder._persist
    if upload_fails:
        finalizer.publisher.publish_video.side_effect = OSError()
    recorder._finalize = lambda document: finalizer.finalize_session(
        document, directory
    )
    cleanup = mocker.patch.object(
        finalizer, "_remove_local_footage", wraps=finalizer._remove_local_footage
    )
    if failure_stage == "publication":
        finalizer.publish.side_effect = OSError()
    else:
        cleanup.side_effect = PermissionError()
    with pytest.raises(OSError):
        asyncio.run(recorder._maintain_session(session))
    assert (directory / "front-00001.mkv").exists() == (failure_stage == "cleanup")

    recovered = DriverRecordingSession.model_validate_json(
        (directory / "session.json").read_text()
    )
    assert recovered.status == ("Expired" if upload_fails else "Ready")
    finalizer.publish.side_effect = None
    cleanup.side_effect = None
    asyncio.run(recorder._maintain_session(recovered))
    finalizer.publisher.publish_video.assert_awaited_once()
    assert finalizer.publish.await_count == (2 if failure_stage == "publication" else 1)
    assert {path.name for path in directory.iterdir()} == {"session.json", "finalized"}


@pytest.mark.parametrize("active", [False, True])
def test_maintenance_only_finalizes_inactive_sessions(recorder, active):
    session = DriverRecordingSession(
        run_id="mission",
        status="Processing",
        cameras={},
        ended_at=(datetime.now(UTC) - timedelta(days=8)).isoformat(),
    )
    recorder._persist(session)
    directory = recorder._session_directory(session)
    footage = directory / "segment.mkv"
    footage.write_bytes(b"footage")
    if active:
        recorder.workers["mission"] = Mock()

    asyncio.run(recorder._maintain_session(session))

    assert footage.exists()
    assert session.status == "Processing"
    assert (directory / "session.json").exists()
    assert recorder._finalize.await_count == int(not active)


def test_finalization_builds_segment_timings_across_reconnect_and_dropped_segment(
    tmp_path, mocker
):
    def convert_side_effect(source, output, **options):
        if source.name == "front-00002.mkv":
            raise RuntimeError("corrupt")
        output.write_bytes(b"video")

    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.convert_segment",
        new=AsyncMock(side_effect=convert_side_effect),
    )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.concatenate_segments",
        new=AsyncMock(side_effect=lambda _, output: output.write_bytes(b"video")),
    )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.validate_video", new=AsyncMock()
    )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.probe_video_duration",
        new=AsyncMock(side_effect=[12.5, 8.0]),
    )
    for name in ("front-00001.mkv", "front-00002.mkv", "front-00003.mkv"):
        (tmp_path / name).write_bytes(b"footage")
    (tmp_path / "session.json").write_text("{}")

    session = DriverRecordingSession(
        run_id="mission",
        status="Processing",
        cameras={
            "front": DriverCameraRecording(
                camera_id="front",
                key="front",
                status="Processing",
                capture_metadata=CaptureMetadata(source_codec="H264"),
                recorded_segments=[
                    RecordedSegment(
                        segment_index=1,
                        first_frame_at="2026-11-27T10:00:01+00:00",
                    ),
                    RecordedSegment(
                        segment_index=2,
                        first_frame_at="2026-11-27T10:00:20+00:00",
                    ),
                    RecordedSegment(
                        segment_index=3,
                        first_frame_at="2026-11-27T10:00:45+00:00",
                    ),
                ],
            )
        },
    )
    publisher = Mock(publish_video=AsyncMock(return_value="mission/front.mp4"))
    finalizer = SessionFinalizer(publisher, Mock(), AsyncMock())

    asyncio.run(finalizer.finalize_session(session, tmp_path))

    assert session.cameras["front"].segment_timings == [
        SegmentTiming(
            segment_index=1,
            video_start_seconds=0.0,
            duration_seconds=12.5,
            capture_start_at="2026-11-27T10:00:01+00:00",
        ),
        SegmentTiming(
            segment_index=3,
            video_start_seconds=12.5,
            duration_seconds=8.0,
            capture_start_at="2026-11-27T10:00:45+00:00",
        ),
    ]
    assert any("front-00002.mkv" in gap.reason for gap in session.gaps)


def test_recorder_attaches_passive_pose_samples_on_finalization(
    tmp_path, telemetry_service, map_handler, mocker
):
    mocker.patch.object(
        API, "get_all_sensor_values", return_value=build_snapshot()
    )
    telemetry_service.refresh_sensor_cache()

    async def capture(stop, open_segment, update_camera):
        segment = open_segment("front", CaptureMetadata(source_codec="H264"))
        segment.write_bytes(b"footage")
        update_camera("front", CaptureMetadata(last_frame_at=100.0))
        telemetry_service.refresh_sensor_cache()
        await asyncio.sleep(0.05)
        stop.set()

    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.convert_segment",
        new=AsyncMock(side_effect=lambda _, out, **__: out.write_bytes(b"video")),
    )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.concatenate_segments",
        new=AsyncMock(side_effect=lambda _, out: out.write_bytes(b"video")),
    )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.validate_video", new=AsyncMock()
    )
    mocker.patch(
        "isar_taurob.robot.recording.session_finalizer.probe_video_duration",
        new=AsyncMock(return_value=5.0),
    )

    published_sessions = []
    published_ready = threading.Event()

    async def record_publish(session):
        published_sessions.append(session.model_copy(deep=True))
        if session.status == "Ready":
            published_ready.set()

    publisher = Mock(
        publish_video=AsyncMock(return_value="mission/front.mp4"),
        publish_session=AsyncMock(side_effect=record_publish),
    )
    recorder = TaurobMissionRecorder(
        capture,
        publisher,
        directory=tmp_path,
        telemetry=telemetry_service,
        map_handler=map_handler,
    )
    try:
        recorder.start("mission-1", "Inspection")
        assert published_ready.wait(timeout=5)
    finally:
        recorder.close()

    ready_sessions = [s for s in published_sessions if s.status == "Ready"]
    assert ready_sessions
    final_session = ready_sessions[-1]
    assert final_session.pose_samples is not None
    assert len(final_session.pose_samples) == 2
    assert all(sample.map_name == "test_map" for sample in final_session.pose_samples)
    assert final_session.cameras["front"].segment_timings is not None
    assert len(final_session.cameras["front"].segment_timings) == 1
    assert final_session.cameras["front"].segment_timings[0].capture_start_at is not None
    session_dir = recorder._session_directory(final_session)
    assert not (session_dir / POSE_LOG_FILENAME).exists()
