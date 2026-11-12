from dataclasses import dataclass, field
from enum import Enum

from isar_taurob.robot.api.acoustic_metadata import ACOUSTIC_MEASUREMENT_DURATION


class SensorName(Enum):
    Visual = "camera_sensorhead"
    Thermal = "camera_thermal_g300a"
    Audio = "microphone"
    Acoustic = "camera_acoustic"


class ResultType(Enum):
    Image = "still"
    Video = "video"
    Audio = "audio"
    ThermalVideo = "thermal-sequence"
    LeakDetection = "leak-detection"


class ResultFormat(Enum):
    Image = "image/jpeg"
    Video = "video/mp4"
    Audio = "audio/mp4"
    ThermalVideo = "application/flir-seq"


@dataclass
class MeasurementMetadata:
    sensor_name: SensorName
    result_type: ResultType
    result_format: ResultFormat
    duration: float = field(default=0)

    @staticmethod
    def dict_factory(data):
        return {
            field: value.value if isinstance(value, Enum) else value
            for field, value in data
        }


@dataclass
class AcousticMeasurementRequest(MeasurementMetadata):
    duration: float = ACOUSTIC_MEASUREMENT_DURATION
    sensitivity: str = "high"
    lights: bool = False
