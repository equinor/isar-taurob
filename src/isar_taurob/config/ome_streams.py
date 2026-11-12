"""Configuration parsing for approved Taurob OME camera streams."""


def is_valid_camera_id(camera_id: str) -> bool:
    """Accept a nonempty URL path segment without /, ?, or # (e.g. front or Arm.1)."""
    return bool(camera_id) and not any(delimiter in camera_id for delimiter in "/?#")


def parse_ome_streams(value: str) -> dict[str, int]:
    """Parse approved Fleet camera IDs and their browser display rotations.

    Example input: "front:-90,arm:0".
    """
    streams: dict[str, int] = {}
    if not value.strip():
        raise ValueError(
            "OME_STREAMS must contain at least one camera_id:rotation pair"
        )
    for entry in value.split(","):
        camera_id, separator, rotation = entry.strip().partition(":")
        if not separator or not is_valid_camera_id(camera_id):
            raise ValueError(f"Invalid OME_STREAMS camera ID: {camera_id!r}")
        if camera_id in streams:
            raise ValueError(f"Duplicate OME_STREAMS camera ID: {camera_id!r}")
        try:
            degrees = int(rotation)
        except ValueError as exc:
            raise ValueError(
                f"Invalid rotation for OME_STREAMS camera ID: {camera_id!r}"
            ) from exc
        if degrees not in {-270, -180, -90, 0, 90, 180, 270}:
            raise ValueError(
                f"Unsupported rotation for OME_STREAMS camera ID: {camera_id!r}"
            )
        streams[camera_id] = degrees
    return streams
