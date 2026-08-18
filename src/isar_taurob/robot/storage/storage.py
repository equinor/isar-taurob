import logging
import os
import subprocess
import tempfile

import imageio_ffmpeg
from requests import Response
from robot_interface.models.exceptions.robot_exceptions import (
    RobotRetrieveInspectionException,
)
from robot_interface.models.inspection.inspection import (  # type: ignore
    Audio,
    AudioMetadata,
    Image,
    ImageMetadata,
    Inspection,
    InspectionMetadata,
    ThermalImage,
    ThermalImageMetadata,
    ThermalVideo,
    ThermalVideoMetadata,
    Video,
    VideoMetadata,
)
from robot_interface.models.mission.task import (
    InspectionTask,
    RecordAudio,
    TakeThermalVideo,
    TakeVideo,
)

from isar_taurob.config import settings
from isar_taurob.robot.api.api import API


class Storage:
    def __init__(
        self,
        robot_api: API,
    ):
        self.robot_api: API = robot_api
        self.logger = logging.getLogger(settings.LOGGER_NAME)
        self.thermal_file_types: list[str] = settings.FILE_FORMAT_THERMAL_IMAGE
        self.image_file_types: list[str] = settings.FILE_FORMAT_IMAGE
        self.video_file_types: list[str] = settings.FILE_FORMAT_VIDEO
        self.audio_file_types: list[str] = settings.FILE_FORMAT_AUDIO
        self.thermal_video_file_types: list[str] = settings.FILE_FORMAT_THERMAL_VIDEO

    def get_inspection(
        self,
        instance_id: int,
        inspection_metadata: InspectionMetadata,
        action_id: int,
        task: InspectionTask,
    ) -> Inspection:
        result_data: list = self.robot_api.get_mission_instance_details(
            instance_id=instance_id
        )
        inspection_type: Inspection
        allowed_file_types: list[str]

        measurement_results: list[dict[str, str]] = list(
            filter(
                lambda i: i["mission_step"] is not None
                and i["mission_step"]["step_id"] == action_id,
                result_data,
            )
        )[-1]["measurement_results"]

        if isinstance(inspection_metadata, ImageMetadata):
            inspection_type = Image
            allowed_file_types = self.image_file_types

        elif isinstance(inspection_metadata, ThermalImageMetadata):
            inspection_type = ThermalImage
            allowed_file_types = self.thermal_file_types

        elif isinstance(inspection_metadata, VideoMetadata):
            inspection_type = Video
            allowed_file_types = self.video_file_types

        elif isinstance(inspection_metadata, ThermalVideoMetadata):
            inspection_type = ThermalVideo
            allowed_file_types = self.thermal_video_file_types

        elif isinstance(inspection_metadata, AudioMetadata):
            inspection_type = Audio
            allowed_file_types = self.audio_file_types
        else:
            error_message = f"Unknown inspection type for action '{action_id}' in mission '{instance_id}'"
            self.logger.error(error_message)
            raise RobotRetrieveInspectionException(error_message)

        data: dict[str, str] = self._get_measurement_result(
            measurement_results, allowed_file_types
        )

        file_url: str = data["url"]
        file_type: str = data["media_type"].split("/")[-1].split("-")[-1]
        response: Response = self.robot_api.get_url_data(file_url)

        file_bytes: bytes = response.content

        if (
            inspection_type is Video
            and file_type == "mp4"
            and settings.NORMALIZE_VIDEO_TO_CONSTANT_FRAME_RATE
        ):
            file_bytes = self._normalize_video_to_constant_frame_rate(file_bytes)

        inspection: Inspection = inspection_type(
            metadata=inspection_metadata, id=task.id, data=file_bytes
        )
        inspection.metadata.inspection_description = task.inspection_description
        inspection.metadata.file_type = file_type

        if isinstance(task, (TakeVideo, TakeThermalVideo, RecordAudio)):
            duration = task.duration
        else:
            duration = -1.0

        if isinstance(
            inspection.metadata,
            (VideoMetadata, ThermalVideoMetadata, AudioMetadata),
        ):
            inspection.metadata.duration = duration

        return inspection

    def _normalize_video_to_constant_frame_rate(self, video_bytes: bytes) -> bytes:
        """Re-encode a variable-frame-rate mp4 to a constant frame rate."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            in_path: str = os.path.join(tmp_dir, "in.mp4")
            out_path: str = os.path.join(tmp_dir, "out.mp4")
            with open(in_path, "wb") as in_file:
                in_file.write(video_bytes)

            try:
                ffmpeg_exe: str = imageio_ffmpeg.get_ffmpeg_exe()
                subprocess.run(
                    [
                        ffmpeg_exe,
                        "-y",
                        "-i",
                        in_path,
                        "-vf",
                        f"fps={settings.VIDEO_TARGET_FRAME_RATE}",
                        "-fps_mode",
                        "cfr",
                        "-c:v",
                        "libx264",
                        "-profile:v",
                        "baseline",
                        "-pix_fmt",
                        "yuv420p",
                        "-movflags",
                        "+faststart",
                        out_path,
                    ],
                    check=True,
                    capture_output=True,
                )
            except (subprocess.CalledProcessError, FileNotFoundError, OSError) as e:
                self.logger.warning(
                    "Failed to normalize video to a constant frame rate, "
                    f"uploading the original bytes instead. Error: {e}"
                )
                return video_bytes

            with open(out_path, "rb") as out_file:
                return out_file.read()

    def _get_measurement_result(
        self, measurement_results: list[dict[str, str]], allowed_file_types
    ) -> dict[str, str]:
        valid_measurement_results: list[dict[str, str]] = list(
            filter(
                lambda i: i["media_type"].split("/")[-1].split("-")[-1]
                in allowed_file_types,
                measurement_results,
            )
        )

        if not valid_measurement_results:
            error_message = (
                f"No measurement result found for file types '{allowed_file_types}'"
            )
            self.logger.error(error_message)
            raise RobotRetrieveInspectionException(error_message)

        if len(valid_measurement_results) > 1:
            self.logger.warning(
                f"Multiple measurement results found for file types '{allowed_file_types}'. Only first will be uploaded"
            )

        return valid_measurement_results[0]
