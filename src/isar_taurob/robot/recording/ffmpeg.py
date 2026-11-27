"""Asynchronous FFmpeg execution for Taurob recording finalization."""

import asyncio
import json
import math
from pathlib import Path

from isar_taurob.robot.recording.subprocess_utils import create_nice_subprocess


async def convert_segment(
    source: Path,
    output: Path,
    *,
    passthrough: bool,
    rotation_degrees: int,
) -> None:
    """Remux compatible footage or transcode it with the requested rotation."""
    filters = {
        90: "transpose=1",
        180: "hflip,vflip",
        270: "transpose=2",
    }
    options = (
        ["-c:v", "copy"]
        if passthrough
        else [
            "-vf",
            filters.get(rotation_degrees % 360, "null"),
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
        ]
    )
    await _execute_ffmpeg(
        "-fflags",
        "+genpts",
        "-i",
        str(source),
        "-an",
        *options,
        "-movflags",
        "+faststart",
        str(output),
    )


async def concatenate_segments(manifest: Path, output: Path) -> None:
    """Combine the manifest's converted segments without re-encoding."""
    await _execute_ffmpeg(
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        str(manifest),
        "-c",
        "copy",
        "-movflags",
        "+faststart",
        str(output),
    )


async def validate_video(path: Path) -> None:
    """Decode the complete video and fail on decoding errors."""
    await _execute_ffmpeg("-xerror", "-i", str(path), "-f", "null", "-")


async def probe_video_duration(path: Path) -> float:
    """Return the video's duration in seconds using ffprobe."""
    process = await create_nice_subprocess(
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=duration:format=duration",
        "-of",
        "json",
        str(path),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    try:
        stdout, _ = await asyncio.wait_for(process.communicate(), 60)
    except BaseException:
        process.kill()
        await process.wait()
        raise
    if process.returncode:
        raise RuntimeError("Video duration probe failed")
    return _parse_probed_duration(stdout)


def _parse_probed_duration(stdout: bytes) -> float:
    payload = json.loads(stdout.decode("utf-8"))
    streams = payload.get("streams") or []
    candidates = [
        streams[0].get("duration") if streams else None,
        (payload.get("format") or {}).get("duration"),
    ]
    for raw in candidates:
        if raw is None or raw == "N/A":
            continue
        duration = float(raw)
        if math.isfinite(duration) and duration >= 0:
            return duration
    raise RuntimeError("Video duration unavailable")


async def _execute_ffmpeg(*arguments: str) -> None:
    process = await create_nice_subprocess(
        "ffmpeg",
        "-nostdin",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        *arguments,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.PIPE,
    )
    try:
        await asyncio.wait_for(process.communicate(), 300)
    except BaseException:
        process.kill()
        await process.wait()
        raise
    if process.returncode:
        # Never include third-party diagnostics containing credentials in metadata.
        raise RuntimeError("Video finalization failed")
