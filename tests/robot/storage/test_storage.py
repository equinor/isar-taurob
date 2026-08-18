import re
import subprocess

import imageio_ffmpeg

from isar_taurob.config import settings


def _make_video(fps: int, duration: int) -> bytes:
    """Create a small H.264 mp4 test clip and return its bytes."""
    ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()
    result = subprocess.run(
        [
            ffmpeg_exe,
            "-y",
            "-f",
            "lavfi",
            "-i",
            f"testsrc=duration={duration}:size=128x128:rate={fps}",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-f",
            "mp4",
            "-movflags",
            "+frag_keyframe+empty_moov",
            "pipe:1",
        ],
        check=True,
        capture_output=True,
    )
    return result.stdout


def _count_frames(video_bytes: bytes) -> int:
    """Count the number of video frames in the given mp4 bytes."""
    ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()
    result = subprocess.run(
        [ffmpeg_exe, "-i", "pipe:0", "-map", "0:v:0", "-f", "null", "-"],
        input=video_bytes,
        capture_output=True,
        check=False,
    )
    stderr = result.stderr.decode("utf-8", errors="ignore")
    matches = re.findall(r"frame=\s*(\d+)", stderr)
    if not matches:
        raise AssertionError(f"Could not determine frame count from:\n{stderr}")
    return int(matches[-1])


def test_normalize_video_reencodes_to_target_frame_rate(storage):
    duration = 2
    source_fps = 10
    input_bytes = _make_video(fps=source_fps, duration=duration)

    output_bytes = storage._normalize_video_to_constant_frame_rate(input_bytes)

    assert output_bytes
    assert output_bytes != input_bytes

    expected_frames = duration * settings.VIDEO_TARGET_FRAME_RATE
    actual_frames = _count_frames(output_bytes)
    # Allow a small tolerance for encoder start/end frame handling.
    assert abs(actual_frames - expected_frames) <= 1


def test_normalize_video_falls_back_to_original_bytes_on_failure(storage, mocker):
    input_bytes = b"not-a-real-video"

    mocker.patch(
        "isar_taurob.robot.storage.storage.subprocess.run",
        side_effect=subprocess.CalledProcessError(returncode=1, cmd="ffmpeg"),
    )

    output_bytes = storage._normalize_video_to_constant_frame_rate(input_bytes)

    assert output_bytes == input_bytes


def test_normalize_video_falls_back_when_ffmpeg_missing(storage, mocker):
    input_bytes = b"not-a-real-video"

    mocker.patch(
        "isar_taurob.robot.storage.storage.subprocess.run",
        side_effect=FileNotFoundError("ffmpeg not found"),
    )

    output_bytes = storage._normalize_video_to_constant_frame_rate(input_bytes)

    assert output_bytes == input_bytes
