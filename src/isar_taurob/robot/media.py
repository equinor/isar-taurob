"""Signed OME playback URLs for approved Taurob camera roles."""

import base64
import hashlib
import hmac
import json
import logging
import time
from urllib.parse import urlsplit

from robot_interface.models.robots.media import (
    MediaConfig,
    MediaConnectionType,
    MediaStream,
)

from isar_taurob.config import settings
from isar_taurob.config.ome_streams import parse_ome_streams

logger = logging.getLogger(settings.LOGGER_NAME)


def encode_base64url(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).decode("ascii").rstrip("=")


def are_ome_signing_settings_configured() -> bool:
    if not settings.OME_PUBLIC_URL:
        logger.warning("Cannot create OME media URL: OME_PUBLIC_URL is not configured")
        return False
    if not settings.OME_SIGNATURE_URL:
        logger.warning(
            "Cannot create OME media URL: OME_SIGNATURE_URL is not configured"
        )
        return False
    if not settings.OME_SIGNED_POLICY_SECRET:
        logger.warning(
            "Cannot create OME media URL: OME_SIGNED_POLICY_SECRET is not configured"
        )
        return False
    return True


def create_signed_ome_url(
    camera_id: str, robot_name: str, now_ms: int | None = None
) -> str | None:
    """Sign the upstream WS URL that OME sees after Gateway TLS termination."""
    if not are_ome_signing_settings_configured():
        return None
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms
    policy = {
        "url_expire": now_ms + settings.OME_URL_TTL_SECONDS * 1000,
        "stream_expire": now_ms + settings.OME_STREAM_TTL_SECONDS * 1000,
    }
    encoded_policy = encode_base64url(
        json.dumps(policy, separators=(",", ":")).encode("utf-8")
    )
    path = f"/{robot_name}/{camera_id}"
    public_url = f"{settings.OME_PUBLIC_URL.rstrip('/')}{path}?policy={encoded_policy}"
    upstream_url = (
        f"{settings.OME_SIGNATURE_URL.rstrip('/')}{path}?policy={encoded_policy}"
    )
    signature = encode_base64url(
        hmac.new(
            settings.OME_SIGNED_POLICY_SECRET.encode("utf-8"),
            upstream_url.encode("utf-8"),
            hashlib.sha1,
        ).digest()
    )
    return f"{public_url}&signature={signature}"


def is_expected_camera_stream_url(url: str, robot_name: str, camera_id: str) -> bool:
    """Accept only Fleet WebSocket URLs matching this robot and camera ID.

    Example: ws://x.x.x.x:8080/hercules/arm.
    """
    try:
        parsed = urlsplit(url)
        return (
            parsed.scheme == "ws"
            and parsed.hostname is not None
            and parsed.username is None
            and parsed.password is None
            and parsed.path == f"/{robot_name}/{camera_id}"
            and not parsed.query
            and not parsed.fragment
        )
    except ValueError:
        return False


def create_ome_media_config(
    video_streams: list[dict], robot_name: str
) -> MediaConfig | None:
    """Sign only configured cameras with unique matching Fleet WebSocket URLs."""
    configured_streams = parse_ome_streams(settings.OME_STREAMS)
    fleet_streams: dict[str, list[object]] = {
        camera_id: [] for camera_id in configured_streams
    }
    for stream in video_streams:
        if not isinstance(stream, dict):
            continue
        camera_id = stream.get("role")  # Fleet names its camera identifier "role".
        if not isinstance(camera_id, str) or camera_id not in configured_streams:
            continue
        fleet_url = stream.get("url")
        # Count all entries for a configured camera so duplicates cannot select an
        # arbitrary endpoint, even when one of the entries is malformed.
        fleet_streams[camera_id].append(fleet_url)

    streams = []
    for camera_id, rotation_degrees in configured_streams.items():
        endpoints = fleet_streams[camera_id]
        if len(endpoints) != 1:
            continue
        fleet_url = endpoints[0]
        if not isinstance(fleet_url, str) or not is_expected_camera_stream_url(
            fleet_url, robot_name, camera_id
        ):
            continue
        signed_url = create_signed_ome_url(camera_id, robot_name)
        if signed_url is None:
            return None
        streams.append(
            MediaStream(
                camera_id=camera_id,
                url=signed_url,
                rotation_degrees=rotation_degrees,
            )
        )
    if not streams:
        return None
    return MediaConfig(
        media_connection_type=MediaConnectionType.OvenMediaEngine, streams=streams
    )
