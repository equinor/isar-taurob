import json
import logging
import os
import subprocess
import tempfile
from decimal import Decimal

import imageio_ffmpeg
from pydantic import ValidationError
from requests import Response
from robot_interface.models.exceptions.robot_exceptions import (
    RobotRetrieveInspectionException,
)
from robot_interface.models.inspection.inspection import (  # type: ignore
    AcousticMeasurement,
    Audio,
    AudioMetadata,
    CO2Measurement,
    COMeasurement,
    GasMeasurement,
    GasMeasurementMetadata,
    H2SMeasurement,
    Image,
    ImageMetadata,
    Inspection,
    InspectionMetadata,
    IsobutyleneMeasurement,
    O2Measurement,
    ThermalImage,
    ThermalImageMetadata,
    ThermalVideo,
    ThermalVideoMetadata,
    TolueneMeasurement,
    Video,
    VideoMetadata,
)
from robot_interface.models.mission.task import (
    InspectionTask,
    RecordAudio,
    TakeAcousticMeasurement,
    TakeThermalVideo,
    TakeVideo,
    TakeXam8000GasMeasurement,
)

from isar_taurob.config import settings
from isar_taurob.robot.api.acoustic_metadata import AcousticMetadata
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.sensor import SensorName
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry

# The gas sensor reports one reading per gas species. The keys are the
# species names used by the X-am 8000 sensor.
GAS_MEASUREMENT_TYPES: dict[str, type[GasMeasurement]] = {
    "IBUT": IsobutyleneMeasurement,
    "CO2": CO2Measurement,
    "TOLU": TolueneMeasurement,
    "O2": O2Measurement,
    "CO": COMeasurement,
    "H2S": H2SMeasurement,
}


def _get_decimal_places(value: object) -> int | None:
    if isinstance(value, int):
        return 0
    if isinstance(value, Decimal) and value.is_finite():
        return max(0, -int(value.as_tuple().exponent))
    return None


class Storage:
    def __init__(
        self,
        robot_api: API,
        robot_telemetry: TaurobTelemetry,
    ):
        self.robot_api: API = robot_api
        self.robot_telemetry: TaurobTelemetry = robot_telemetry
        self.logger = logging.getLogger(settings.LOGGER_NAME)
        self.thermal_file_types: list[str] = settings.FILE_FORMAT_THERMAL_IMAGE
        self.image_file_types: list[str] = settings.FILE_FORMAT_IMAGE
        self.video_file_types: list[str] = settings.FILE_FORMAT_VIDEO
        self.audio_file_types: list[str] = settings.FILE_FORMAT_AUDIO
        self.thermal_video_file_types: list[str] = settings.FILE_FORMAT_THERMAL_VIDEO

    def get_inspections(
        self,
        instance_id: int,
        inspection_metadata: InspectionMetadata,
        action_id: int,
        task: InspectionTask,
    ) -> list[Inspection]:
        if isinstance(task, TakeXam8000GasMeasurement):
            if not isinstance(inspection_metadata, GasMeasurementMetadata):
                error_message = (
                    f"Expected gas measurement metadata for task '{task.id}', "
                    f"got '{type(inspection_metadata).__name__}'"
                )
                self.logger.error(error_message)
                raise RobotRetrieveInspectionException(error_message)
            return self._get_gas_measurement(task, inspection_metadata)

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

        if isinstance(task, TakeAcousticMeasurement):
            return [
                self._get_acoustic_inspection(
                    measurement_results, inspection_metadata, task
                )
            ]

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
            inspection_type in (Video, ThermalVideo)
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

        return [inspection]

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

    def _get_gas_measurement(
        self,
        task: TakeXam8000GasMeasurement,
        inspection_metadata: GasMeasurementMetadata,
    ) -> list[GasMeasurement]:
        gas_values_json: str = self.robot_telemetry._get_value(
            SensorName.GAS_CONCENTRATION
        )

        try:
            # Expected format: {"gas_type_shortname": {"value": float, "unit": str}}
            all_gas_values_dict: dict = json.loads(gas_values_json, parse_float=Decimal)
        except (TypeError, json.JSONDecodeError) as e:
            error_message = (
                f"Could not decode the gas measurement for task '{task.id}': {e}"
            )
            self.logger.error(error_message)
            raise RobotRetrieveInspectionException(error_message) from e

        measurements: list[GasMeasurement] = []
        for gas_type_shortname, measurement_type in GAS_MEASUREMENT_TYPES.items():
            gas_value_dict: dict | None = all_gas_values_dict.get(gas_type_shortname)
            if gas_value_dict is None:
                continue
            try:
                raw_value = gas_value_dict["value"]
                measurement: GasMeasurement = measurement_type(
                    id=task.id,
                    value=float(raw_value),
                    unit=str(gas_value_dict["unit"]),
                    decimal_places=_get_decimal_places(raw_value),
                    metadata=inspection_metadata,
                )
            except (KeyError, TypeError, ValueError) as e:
                error_message = f"Invalid gas reading for '{gas_type_shortname}' in task '{task.id}': {e}"
                self.logger.error(error_message)
                raise RobotRetrieveInspectionException(error_message) from e
            measurements.append(measurement)

        if not measurements:
            error_message = (
                f"The gas sensor reported no known gas species for task '{task.id}'"
            )
            self.logger.error(error_message)
            raise RobotRetrieveInspectionException(error_message)

        return measurements

    def _get_acoustic_inspection(
        self,
        measurement_results: list[dict[str, str]],
        inspection_metadata: InspectionMetadata,
        task: TakeAcousticMeasurement,
    ) -> AcousticMeasurement:
        metadata_files = [
            json.loads(self.robot_api.get_url_data(result["url"]).content)
            for result in measurement_results
            if result["media_type"] == "application/json"
        ]
        version_1 = [data for data in metadata_files if data.get("version") == 1]
        videos = [
            r["url"] for r in measurement_results if r["media_type"] == "video/mp4"
        ]

        if len(version_1) != 1 or len(videos) != 1:
            error_message = (
                "Expected one version 1 metadata file and one video "
                f"for acoustic task '{task.id}'"
            )
            self.logger.error(error_message)
            raise RobotRetrieveInspectionException(error_message)

        try:
            acoustic_metadata = AcousticMetadata.model_validate(version_1[0])
        except ValidationError as e:
            error_message = f"Invalid acoustic metadata for task '{task.id}': {e}"
            self.logger.error(error_message)
            raise RobotRetrieveInspectionException(error_message) from e

        return AcousticMeasurement(
            id=task.id,
            data=self.robot_api.get_url_data(videos[0]).content,
            metadata=acoustic_metadata.to_inspection_metadata(inspection_metadata),
        )

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
