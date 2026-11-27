"""Taurob background recording sessions with recoverable segments and uploads."""

import asyncio
import hashlib
import logging
import threading
from datetime import UTC, datetime
from pathlib import Path

from robot_interface.mission_recording import MissionRecorder
from robot_interface.recording_models import PoseSample, RecordingGap
from robot_interface.recording_publisher import RecordingPublisher

from isar_taurob.robot.map.map_handler import MapHandler
from isar_taurob.robot.recording.models import (
    CameraEvent,
    Capture,
    CaptureMetadata,
    DriverCameraRecording,
    DriverRecordingSession,
    RecordedSegment,
)
from isar_taurob.robot.recording.pose_collector import (
    append_pose_sample,
    pose_snapshot_to_sample,
)
from isar_taurob.robot.recording.session_finalizer import SessionFinalizer
from isar_taurob.robot.telemetry.pose_snapshot import PoseSnapshot
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry

logger = logging.getLogger(__name__)


def timestamp() -> str:
    return datetime.now(UTC).isoformat()


class TaurobMissionRecorder(MissionRecorder):
    """A driver's single background loop owns all metadata and media callbacks."""

    telemetry: TaurobTelemetry | None = None
    map_handler: MapHandler | None = None

    def __init__(
        self,
        capture: Capture,
        publisher: RecordingPublisher,
        *,
        directory: Path,
        telemetry: TaurobTelemetry | None = None,
        map_handler: MapHandler | None = None,
    ) -> None:
        """Initialize durable recording storage and start the background event loop."""
        self.capture = capture
        self.publisher = publisher
        self.directory = directory
        self.telemetry = telemetry
        self.map_handler = map_handler
        self.directory.mkdir(parents=True, exist_ok=True)
        self.sessions: dict[str, DriverRecordingSession] = {}
        self.workers: dict[str, tuple[asyncio.Event, asyncio.Task[None]]] = {}
        self.closing = False
        self.ready = threading.Event()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(
            target=self._run_loop, name="Mission recordings", daemon=True
        )
        self.thread.start()
        if not self.ready.wait(5):
            raise RuntimeError("Recording loop did not start")
        if self.telemetry is not None and self.map_handler is not None:
            self.telemetry.add_pose_listener(self._on_pose_snapshot)

    def start(self, mission_id: str, mission_name: str) -> None:
        """Schedule mission recording on the background loop from any thread."""
        self.loop.call_soon_threadsafe(self._start_session, mission_id, mission_name)

    def stop(self, mission_id: str, interrupted: bool = False) -> None:
        """Schedule recording completion on the background loop from any thread."""
        self.loop.call_soon_threadsafe(self._stop_session, mission_id, interrupted)

    def close(self) -> None:
        """Request bounded recorder shutdown and stop its background thread."""
        if self.telemetry is not None:
            self.telemetry.remove_pose_listener(self._on_pose_snapshot)
        if not self.thread.is_alive():
            return
        future = asyncio.run_coroutine_threadsafe(self._shutdown(), self.loop)
        try:
            future.result(timeout=20)
        except Exception:  # noqa: BLE001 - bounded recorder shutdown.
            logger.warning("Recording shutdown incomplete; durable segments retained")
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2)
        if not self.thread.is_alive():
            logger.info("Recording background thread stopped")

    def _run_loop(self) -> None:
        """Recover saved sessions and run capture and maintenance tasks."""
        asyncio.set_event_loop(self.loop)
        for path in self.directory.glob("*/session.json"):
            session = DriverRecordingSession.model_validate_json(path.read_text())
            logger.info(
                "Recording session recovered mission_id=%s previous_status=%s interrupted=%s",
                session.run_id,
                session.status,
                not bool(session.ended_at),
            )
            if not session.ended_at:
                session.status = "Processing"
                session.ended_at = timestamp()
                session.gaps.append(
                    RecordingGap(
                        at=timestamp(),
                        reason="ISAR restarted; mission continuity unknown",
                    )
                )
            self.sessions[session.run_id] = session
            self._persist(session)
        self.maintenance = self.loop.create_task(self._maintain_sessions())
        self.ready.set()
        try:
            self.loop.run_forever()
        finally:
            tasks = asyncio.all_tasks(self.loop)
            for task in tasks:
                task.cancel()
            self.loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
            self.loop.close()

    def _session_directory(self, session: DriverRecordingSession) -> Path:
        """Return the session's stable, filesystem-safe local directory."""
        return self.directory / hashlib.sha256(session.run_id.encode()).hexdigest()

    def _persist(self, session: DriverRecordingSession) -> None:
        """Atomically save the session catalog for recovery after a restart."""
        directory = self._session_directory(session)
        directory.mkdir(exist_ok=True)
        temporary = directory / "session.tmp"
        temporary.write_text(session.model_dump_json(by_alias=True, exclude_none=True))
        temporary.replace(directory / "session.json")

    async def _publish(self, session: DriverRecordingSession) -> None:
        """Publish the current session snapshot through the platform publisher."""
        await self.publisher.publish_session(session)

    def _on_pose_snapshot(self, snapshot: PoseSnapshot) -> None:
        if (
            self.closing
            or not self.workers
            or self.map_handler is None
            or not self.loop.is_running()
        ):
            return
        sample = pose_snapshot_to_sample(snapshot, self.map_handler)
        if sample is None:
            return
        try:
            self.loop.call_soon_threadsafe(self._record_pose_sample, sample)
        except RuntimeError:
            return

    def _record_pose_sample(self, sample: PoseSample) -> None:
        for run_id in self.workers:
            session = self.sessions.get(run_id)
            if session is not None and session.ended_at is None:
                append_pose_sample(self._session_directory(session), sample)

    def _record_initial_cached_pose(self, session: DriverRecordingSession) -> None:
        if self.telemetry is None or self.map_handler is None:
            return
        cached = self.telemetry.get_cached_pose_snapshot()
        if cached is None:
            return
        sample = pose_snapshot_to_sample(cached, self.map_handler)
        if sample is not None:
            append_pose_sample(self._session_directory(session), sample)

    def _start_session(self, mission_id: str, mission_name: str) -> None:
        """Create and launch capture unless the mission is active or already ended."""
        if self.closing or mission_id in self.workers:
            return
        existing = self.sessions.get(mission_id)
        if existing and existing.ended_at:
            return
        session = existing or DriverRecordingSession(
            run_id=mission_id,
            mission_name=mission_name,
            status="Starting",
            started_at=timestamp(),
            mission_started_at=timestamp(),
            cameras={},
        )
        self.sessions[mission_id] = session
        self._persist(session)
        self._record_initial_cached_pose(session)
        stop = asyncio.Event()
        task = asyncio.create_task(self._capture_session(session, stop))
        self.workers[mission_id] = (stop, task)
        logger.info("Recording started mission_id=%s", mission_id)

    def _stop_session(self, mission_id: str, interrupted: bool) -> None:
        """Persist the mission end and signal capture to stop and finalize."""
        session = self.sessions.get(mission_id)
        if not session or session.ended_at:
            return
        session.ended_at = timestamp()
        if interrupted:
            session.gaps.append(
                RecordingGap(at=timestamp(), reason="Mission monitoring interrupted")
            )
        self._persist(session)
        if mission_id in self.workers:
            self.workers[mission_id][0].set()
        logger.info(
            "Recording stop requested mission_id=%s interrupted=%s",
            mission_id,
            interrupted,
        )

    async def _capture_session(
        self, session: DriverRecordingSession, stop: asyncio.Event
    ) -> None:
        """Capture with retries until stopped, then finalize the durable footage."""

        def open_segment(camera: str, metadata: CaptureMetadata) -> Path:
            """Register a camera segment and return its local capture path."""
            recording = session.cameras.setdefault(
                camera,
                DriverCameraRecording(
                    camera_id=camera,
                    key=hashlib.sha256(camera.encode()).hexdigest()[:20],
                    status="Recording",
                ),
            )
            recording.apply_metadata(metadata)
            recording.status = "Recording"
            recording.segments += 1
            recording.recorded_segments.append(
                RecordedSegment(segment_index=recording.segments)
            )
            recording.events.append(CameraEvent(type="started", at=timestamp()))
            logger.debug(
                "Recording segment opened mission_id=%s camera_id=%s segment=%s codec=%s",
                session.run_id,
                camera,
                recording.segments,
                recording.capture_metadata.source_codec,
            )
            session.status = "Recording"
            session.error = None
            self._persist(session)
            return (
                self._session_directory(session)
                / f"{recording.key}-{recording.segments:05}.mkv"
            )

        def update_camera(camera: str, metadata: CaptureMetadata) -> None:
            """Apply capture metadata and mark the camera's first frame arrival."""
            recording = session.cameras[camera]
            now = timestamp()
            segment_updated = False
            if recording.first_frame_at is None:
                recording.first_frame_at = now
                logger.info(
                    "First recording footage received mission_id=%s camera_id=%s",
                    session.run_id,
                    camera,
                )
            if (
                recording.recorded_segments
                and recording.recorded_segments[-1].first_frame_at is None
            ):
                recording.recorded_segments[-1].first_frame_at = now
                segment_updated = True
            recording.apply_metadata(metadata)
            if segment_updated:
                self._persist(session)

        try:
            while not stop.is_set():
                try:
                    await self.capture(stop, open_segment, update_camera)
                except Exception as error:  # noqa: BLE001 - retry capture failures.
                    reason = f"Native receiver failed ({type(error).__name__})"
                    repeated_failure = session.error == reason
                    session.status = "Reconnecting"
                    session.error = reason
                    session.gaps.append(RecordingGap(at=timestamp(), reason=reason))
                    self._persist(session)
                    if not repeated_failure:
                        logger.warning(
                            "Recording capture failed mission_id=%s error_type=%s; reconnecting",
                            session.run_id,
                            type(error).__name__,
                        )
                if not stop.is_set():
                    await asyncio.sleep(2)
            session.status = "Processing"
            session.ended_at = session.ended_at or timestamp()
            self._persist(session)
            await self._finalize(session)
        finally:
            self.workers.pop(session.run_id, None)

    async def _finalize(self, session: DriverRecordingSession) -> None:
        """Delegate media finalization and publication to the session finalizer."""
        finalizer = SessionFinalizer(self.publisher, self._persist, self._publish)
        await finalizer.finalize_session(session, self._session_directory(session))

    async def _maintain_sessions(self) -> None:
        """Periodically retry session publication and local footage cleanup."""
        previous_failures: dict[str, str] = {}
        while True:
            for session in list(self.sessions.values()):
                try:
                    await self._maintain_session(session)
                    previous_failures.pop(session.run_id, None)
                except Exception as error:  # noqa: BLE001 - retry cleanup.
                    error_type = type(error).__name__
                    if previous_failures.get(session.run_id) != error_type:
                        logger.warning(
                            "Recording maintenance will retry mission_id=%s error_type=%s",
                            session.run_id,
                            error_type,
                        )
                    previous_failures[session.run_id] = error_type
            await asyncio.sleep(5)

    async def _maintain_session(self, session: DriverRecordingSession) -> None:
        """Finalize ended sessions once, retrying only publication and cleanup."""
        if (
            session.status in ("Processing", "Failed", "Ready", "Expired")
            and session.ended_at
            and session.run_id not in self.workers
        ):
            await self._finalize(session)
            return
        await self._publish(session)

    async def _shutdown(self) -> None:
        """Stop active captures with a bounded wait and cancel session maintenance."""
        self.closing = True
        active = list(self.workers.items())
        for mission_id, _ in active:
            self._stop_session(mission_id, True)
        if active:
            await asyncio.wait([worker[1][1] for worker in active], timeout=15)
        self.maintenance.cancel()
        await asyncio.gather(self.maintenance, return_exceptions=True)
