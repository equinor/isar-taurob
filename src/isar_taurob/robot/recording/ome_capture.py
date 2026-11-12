"""Taurob-owned OME viewer signalling and encoded video reception."""

import asyncio
import json
import time
from collections.abc import Callable
from importlib import import_module
from pathlib import Path
from typing import Any
from urllib.parse import quote

from robot_interface.models.robots.media import MediaConfig, MediaStream
from websockets.asyncio.client import connect

from isar_taurob.robot.recording.models import (
    CaptureMetadata,
    OpenSegment,
    UpdateCamera,
)

CODEC_CHAINS = {
    "H264": "rtph264depay ! h264parse config-interval=-1",
    "VP8": "rtpvp8depay",
    "VP9": "rtpvp9depay ! vp9parse",
}
SEGMENT_ALLOCATION_TIMEOUT = 10
SIGNALLING_OPEN_TIMEOUT = 15
PACKET_STALL_TIMEOUT = 30
RECEIVER_POLL_INTERVAL = 0.2
PIPELINE_FLUSH_TIMEOUT = 5


def _load_gstreamer() -> tuple[Any, Any, Any]:
    """Load optional native bindings only when starting a recording receiver."""
    gi = import_module("gi")
    for namespace in ("Gst", "GstWebRTC", "GstSdp"):
        gi.require_version(namespace, "1.0")
    Gst = import_module("gi.repository.Gst")
    GstSdp = import_module("gi.repository.GstSdp")
    GstWebRTC = import_module("gi.repository.GstWebRTC")
    Gst.init(None)
    return Gst, GstSdp, GstWebRTC


async def _cancel_capture_tasks(tasks: list[asyncio.Task]) -> None:
    """Cancel and drain workers before releasing their media resources."""
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


class _OMEStreamReceiver:
    """Own one camera's signalling and pipeline; bridge native callbacks to asyncio."""

    def __init__(
        self,
        stream: MediaStream,
        stop: asyncio.Event,
        open_segment: OpenSegment,
        update_camera: UpdateCamera,
    ) -> None:
        """Create per-stream state on the recording worker's asyncio loop."""
        self.stream = stream
        self.stop = stop
        self.open_segment = open_segment
        self.update_camera = update_camera
        self.gst, self.gst_sdp, self.gst_webrtc = _load_gstreamer()
        self.loop = asyncio.get_running_loop()
        self.failure: asyncio.Future[RuntimeError] = self.loop.create_future()
        self.outgoing: asyncio.Queue[dict] = asyncio.Queue()
        self.last_packet_at = time.monotonic()
        self.received_first_packet = False
        self.description = CaptureMetadata()
        self.identifiers: dict = {}
        self.pipeline = self.gst.Pipeline.new(None)
        self.peer = self._create_webrtc_receiver()

    def _create_webrtc_receiver(self) -> Any:
        """Add the WebRTC peer and connect its native-thread callbacks."""
        peer = self.gst.ElementFactory.make("webrtcbin", "receiver")
        if peer is None:
            raise RuntimeError("GStreamer WebRTC plugins unavailable")
        self.pipeline.add(peer)
        peer.connect("on-ice-candidate", self._queue_local_candidate)
        peer.connect("pad-added", self._handle_media_pad)
        return peer

    def _report_pipeline_error(self, message: str) -> None:
        """Publish the first sanitized native failure on the asyncio loop."""
        if not self.failure.done():
            self.failure.set_result(RuntimeError(message))

    def _queue_local_candidate(self, _peer: Any, index: int, candidate: str) -> None:
        """Forward a native ICE callback to the asyncio signalling queue."""
        self.loop.call_soon_threadsafe(
            self.outgoing.put_nowait,
            {
                **self.identifiers,
                "command": "candidate",
                "candidates": [{"sdpMLineIndex": index, "candidate": candidate}],
            },
        )

    async def _allocate_segment(self, codec: str) -> Path:
        """Allocate durable footage and update metadata on the owning loop."""
        self.description.source_codec = codec
        self.description.capture_mode = "encoded-passthrough"
        self.description.rotation_degrees = self.stream.rotation_degrees
        return self.open_segment(
            self.stream.camera_id,
            self.description,
        )

    def _handle_media_pad(self, _peer: Any, pad: Any) -> None:
        """Contain native pad callback errors and notify the asyncio monitor."""
        try:
            self._attach_media_pad(pad)
        except Exception:  # noqa: BLE001 - native callbacks must not leak exceptions.
            self.loop.call_soon_threadsafe(
                self._report_pipeline_error, "Could not attach OME media pipeline"
            )

    def _attach_media_pad(self, pad: Any) -> None:
        """Route native media pads to a discard sink or encoded recording branch."""
        caps = pad.get_current_caps() or pad.query_caps(None)
        structure = caps.get_structure(0)
        if structure.get_string("media") != "video":
            self._discard_media_pad(pad)
            return
        codec = structure.get_string("encoding-name").upper()
        if codec not in CODEC_CHAINS:
            self.loop.call_soon_threadsafe(
                self._report_pipeline_error, "Unsupported OME codec"
            )
            return
        allocated = asyncio.run_coroutine_threadsafe(
            self._allocate_segment(codec), self.loop
        ).result(timeout=SEGMENT_ALLOCATION_TIMEOUT)
        self._attach_recording_branch(pad, codec, allocated)
        pad.add_probe(self.gst.PadProbeType.BUFFER, self._track_received_packet)

    def _discard_media_pad(self, pad: Any) -> None:
        """Consume non-video media so it cannot block the WebRTC pipeline."""
        sink = self.gst.ElementFactory.make("fakesink")
        self.pipeline.add(sink)
        sink.sync_state_with_parent()
        pad.link(sink.get_static_pad("sink"))

    def _attach_recording_branch(self, pad: Any, codec: str, path: Path) -> None:
        """Depayload encoded video into recoverable one-second Matroska clusters."""
        branch = self.gst.parse_bin_from_description(
            f"queue ! {CODEC_CHAINS[codec]} ! "
            "matroskamux streamable=true max-cluster-duration=1000000000 ! "
            f'filesink location="{path}"',
            True,
        )
        self.pipeline.add(branch)
        branch.sync_state_with_parent()
        pad.link(branch.get_static_pad("sink"))

    def _track_received_packet(self, *_args: Any) -> Any:
        """Track native packet arrival and announce the first packet on asyncio."""
        self.last_packet_at = time.monotonic()
        if not self.received_first_packet:
            self.received_first_packet = True
            self.loop.call_soon_threadsafe(
                self.update_camera,
                self.stream.camera_id,
                CaptureMetadata(last_frame_at=time.time()),
            )
        return self.gst.PadProbeReturn.OK

    def _add_remote_candidates(self, message: dict) -> None:
        """Apply candidates from either an offer or a subsequent ICE message."""
        for candidate in message.get("candidates", []):
            self.peer.emit(
                "add-ice-candidate",
                candidate.get("sdpMLineIndex", 0),
                candidate["candidate"],
            )

    def _configure_ice_servers(self, message: dict) -> None:
        """Use advertised TURN credentials and prefer relay when TURN is offered."""
        servers = message.get("iceServers") or message.get("ice_servers") or []
        turn_configured = False
        for server in servers:
            urls = server.get("urls", [])
            for url in [urls] if isinstance(urls, str) else urls:
                if url.startswith(("turn:", "turns:")):
                    self._add_turn_server(server, url)
                    turn_configured = True
        if turn_configured:
            self.peer.set_property(
                "ice-transport-policy", self.gst_webrtc.WebRTCICETransportPolicy.RELAY
            )
        self.description.ice_policy = "relay" if turn_configured else "all"
        self.description.turn_configured = turn_configured

    def _add_turn_server(self, server: dict, url: str) -> None:
        """Translate OME credentials into GStreamer's escaped TURN URI format."""
        scheme, address = url.split(":", 1)
        username = quote(server.get("username") or server.get("user_name", ""), safe="")
        password = quote(server.get("credential", ""), safe="")
        turn = f"{scheme}://{username}:{password}@{address.lstrip('/')}"
        if not self.peer.emit("add-turn-server", turn):
            raise RuntimeError("GStreamer rejected OME TURN configuration")

    async def _apply_remote_offer(self, message: dict) -> None:
        """Install the remote SDP without blocking asyncio on a native promise."""
        _, sdp = self.gst_sdp.SDPMessage.new_from_text(message["sdp"]["sdp"])
        offer = self.gst_webrtc.WebRTCSessionDescription.new(
            self.gst_webrtc.WebRTCSDPType.OFFER, sdp
        )
        promise = self.gst.Promise.new()
        self.peer.emit("set-remote-description", offer, promise)
        await asyncio.to_thread(promise.wait)

    async def _create_local_answer(self) -> Any:
        """Generate and install the local WebRTC session description."""
        promise = self.gst.Promise.new()
        self.peer.emit("create-answer", None, promise)
        await asyncio.to_thread(promise.wait)
        reply = promise.get_reply()
        if reply.has_field("error"):
            raise RuntimeError("GStreamer could not answer OME offer")
        answer = reply.get_value("answer")
        self.peer.emit("set-local-description", answer, self.gst.Promise.new())
        # The reply owns the boxed description; detach it before the promise dies.
        return answer.copy()

    async def _answer_offer(self, message: dict) -> None:
        """Negotiate the offered stream before applying its initial candidates."""
        self.identifiers.update(id=message["id"], peer_id=message.get("peer_id", 0))
        self._configure_ice_servers(message)
        await self._apply_remote_offer(message)
        answer = await self._create_local_answer()
        await self.outgoing.put(
            {
                **self.identifiers,
                "command": "answer",
                "sdp": {"type": "answer", "sdp": answer.sdp.as_text()},
            }
        )
        self._add_remote_candidates(message)

    async def _handle_signalling_message(self, message: dict) -> None:
        """Dispatch OME commands and reject terminal signalling failures."""
        if message.get("error"):
            raise RuntimeError("OME signalling rejected viewer request")
        command = message.get("command")
        if command == "ping":
            await self.outgoing.put({"command": "pong"})
        elif command == "candidate":
            self._add_remote_candidates(message)
        elif command == "stop":
            raise RuntimeError("OME stopped the stream")
        elif command == "offer":
            await self._answer_offer(message)

    async def _send_signalling_messages(self, socket: Any) -> None:
        """Serialize queued answers and local ICE candidates to the viewer socket."""
        while True:
            await socket.send(json.dumps(await self.outgoing.get()))

    async def _receive_signalling_messages(self, socket: Any) -> None:
        """Consume viewer commands and treat socket closure as a capture failure."""
        async for raw in socket:
            await self._handle_signalling_message(json.loads(raw))
        raise RuntimeError("OME signalling connection closed")

    async def _monitor_capture(self, tasks: list[asyncio.Task]) -> None:
        """Watch signalling, native errors and packet stalls until mission stop."""
        bus = self.pipeline.get_bus()
        while not self.stop.is_set():
            if self.failure.done():
                raise self.failure.result()
            for task in tasks:
                if task.done():
                    task.result()
            if bus.pop_filtered(self.gst.MessageType.ERROR):
                raise RuntimeError("GStreamer receiver failed")
            if time.monotonic() - self.last_packet_at > PACKET_STALL_TIMEOUT:
                raise RuntimeError("OME video packets stalled")
            await asyncio.sleep(RECEIVER_POLL_INTERVAL)

    async def _flush_pipeline(self) -> None:
        """Allow muxers a bounded EOS flush before releasing native resources."""
        self.pipeline.send_event(self.gst.Event.new_eos())
        await asyncio.to_thread(
            self.pipeline.get_bus().timed_pop_filtered,
            PIPELINE_FLUSH_TIMEOUT * self.gst.SECOND,
            self.gst.MessageType.EOS | self.gst.MessageType.ERROR,
        )
        self.pipeline.set_state(self.gst.State.NULL)

    async def capture(self) -> None:
        """Run one viewer connection and drain signalling before flushing footage."""
        self.pipeline.set_state(self.gst.State.PLAYING)
        tasks = []
        try:
            async with connect(
                self.stream.url, open_timeout=SIGNALLING_OPEN_TIMEOUT
            ) as socket:
                await socket.send(json.dumps({"command": "request_offer"}))
                tasks = [
                    asyncio.create_task(self._send_signalling_messages(socket)),
                    asyncio.create_task(self._receive_signalling_messages(socket)),
                ]
                await self._monitor_capture(tasks)
        finally:
            await _cancel_capture_tasks(tasks)
            await self._flush_pipeline()


async def capture_ome_stream(
    stream: MediaStream,
    stop: asyncio.Event,
    open_segment: OpenSegment,
    update_camera: UpdateCamera,
) -> None:
    """Capture one approved camera using its signed OME viewer URL."""
    receiver = _OMEStreamReceiver(stream, stop, open_segment, update_camera)
    await receiver.capture()


async def capture_ome(
    generate_config: Callable[[], MediaConfig],
    stop: asyncio.Event,
    open_segment: OpenSegment,
    update_camera: UpdateCamera,
) -> None:
    """Capture approved cameras concurrently and cancel siblings on failure."""
    config = await asyncio.to_thread(generate_config)
    tasks = [
        asyncio.create_task(
            capture_ome_stream(stream, stop, open_segment, update_camera)
        )
        for stream in config.streams or []
    ]
    if not tasks:
        raise RuntimeError("No approved OME cameras")
    try:
        await asyncio.gather(*tasks)
    finally:
        await _cancel_capture_tasks(tasks)
