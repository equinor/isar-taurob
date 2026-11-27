"""Convert Taurob pose snapshots to asset-frame samples and persist them during recording."""

import logging
from datetime import UTC, datetime
from pathlib import Path

from pydantic import ValidationError
from robot_interface.models.exceptions.robot_exceptions import RobotMapException
from robot_interface.recording_models import PoseSample, RecordingPose

from isar_taurob.robot.map.map_handler import MapHandler
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose
from isar_taurob.robot.telemetry.pose_snapshot import PoseSnapshot

logger = logging.getLogger(__name__)

POSE_LOG_FILENAME = "poses.jsonl"


def pose_snapshot_to_sample(
    snapshot: PoseSnapshot, map_handler: MapHandler
) -> PoseSample | None:
    """Transform a Taurob navigation-map snapshot into an asset-frame recording sample."""
    if not snapshot.floor:
        logger.warning("Skipping recording pose sample without floor name")
        return None
    taurob_pose = TaurobPose(
        x=snapshot.x,
        y=snapshot.y,
        theta=snapshot.theta,
        map_name=snapshot.floor,
    )
    try:
        asset_pose = map_handler.transform_pose_to_asset(taurob_pose)
    except RobotMapException as error:
        logger.warning(
            "Skipping recording pose sample for unknown floor=%s: %s",
            snapshot.floor,
            error.error_description,
        )
        return None
    received_at = (snapshot.received_at or datetime.now(UTC)).isoformat()
    source_timestamp = snapshot.changed.isoformat() if snapshot.changed else None
    return PoseSample(
        received_at=received_at,
        source_timestamp=source_timestamp,
        pose=RecordingPose.from_alitra(asset_pose),
        map_name=snapshot.floor,
        confidence=snapshot.confidence,
    )


def append_pose_sample(directory: Path, sample: PoseSample) -> None:
    """Append one serialized pose sample to the session's durable JSONL log."""
    directory.mkdir(exist_ok=True)
    line = sample.model_dump_json(by_alias=True, exclude_none=True)
    with (directory / POSE_LOG_FILENAME).open("a", encoding="utf-8") as handle:
        handle.write(f"{line}\n")


def load_pose_samples(directory: Path) -> list[PoseSample] | None:
    """Load persisted pose samples for a session, or None when no pose log exists."""
    path = directory / POSE_LOG_FILENAME
    if not path.exists():
        return None
    samples: list[PoseSample] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            samples.append(PoseSample.model_validate_json(line))
        except ValidationError:
            logger.warning("Skipping malformed pose sample line in %s", path.name)
    return samples
