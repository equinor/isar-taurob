from dataclasses import dataclass, field
from enum import Enum


class SensorName(Enum):
    Visual = "camera_sensorhead"
    Thermal = "camera_thermal_g300a"
    Audio = "microphone"


class ResultType(Enum):
    Image = "still"
    Video = "video"
    Audio = "audio"
    ThermalVideo = "thermal-sequence"


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
