from importlib.resources import as_file, files
from pathlib import Path
from typing import Literal

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

from isar_taurob.config.ome_streams import parse_ome_streams


class Settings(BaseSettings):
    def __init__(self) -> None:
        try:
            source = files("isar_taurob").joinpath("config").joinpath("settings.env")
            with as_file(source) as eml:
                env_file = eml
        except ModuleNotFoundError:
            env_file = None
        super().__init__(_env_file=env_file)

    # Name of asset is needed for the map configs to be read
    # from the correct folder.
    ASSET_SHORT_NAME: str = Field(default="default")

    # Name of the robot which is being controlled
    # Correct name is important for correct API calls
    ROBOT_NAME: str = Field(default="william")

    RECORDING_DATA_DIRECTORY: Path = Field(
        default=Path("./recordings"),
        validation_alias="ISAR_RECORDING_DATA_DIRECTORY",
    )

    # Ensuring ROBOT_NAME is always lower case
    @field_validator("ROBOT_NAME")
    def lowercase_robot_name(cls, v):
        return v.lower()

    # Name used for the loggers within the isar-taurob package
    LOGGER_NAME: str = Field(default="isar_taurob")

    # Credentials for the API
    API_USERNAME: str = Field(default="username")
    API_PASSWORD: str = Field(default="password")

    # Host for Taurob API
    ROBOT_API_HOST: str = Field(default="https://10.1.0.1:8443/basic")
    API_PROXY: str = Field(
        default="socks5://localhost:1080"
    )  # We assume this is created, e.g., by wireproxy

    # Public WSS URL returned to clients for playback.
    OME_PUBLIC_URL: str = Field(default="")
    # WS URL OME sees internally in the cluster and
    # used to calculate the signature for playback URLs.
    OME_SIGNATURE_URL: str = Field(default="")
    # Shared key for signing playback URLs; must match OME's key.
    OME_SIGNED_POLICY_SECRET: str = Field(default="")
    # Time allowed to start playback with a signed URL.
    OME_URL_TTL_SECONDS: int = Field(default=60)
    # Maximum duration of an authorized stream.
    OME_STREAM_TTL_SECONDS: int = Field(default=3600)
    # Comma-separated approved Fleet camera IDs and browser display rotations.
    OME_STREAMS: str = Field(default="front:-90")

    @field_validator("OME_STREAMS")
    @classmethod
    def validate_ome_streams(cls, value: str) -> str:
        parse_ome_streams(value)
        return value

    # API endpoints
    ROBOT_MISSION_DEFINITION_ENDPOINT: str = Field(default="/fleet/mission/definition")
    ROBOT_MISSION_SCHEDULE_ENDPOINT: str = Field(default="/fleet/mission/schedule")
    ROBOT_ACTION_ENDPOINT: str = Field(default="/fleet/robot-action")
    ROBOT_ENDPOINT: str = Field(default="/fleet/robot")
    ROBOT_MISSION_EDITOR_ENDPOINT: str = Field(default="/mission_editor/data_model")

    # Timeout in seconds for all requests made to the Taurob API
    ROBOT_REQUESTS_TIMEOUT: float = Field(default=60)

    MAX_RETRIES_TELEMETRY: int = Field(default=10)

    # Delay to give enough time for the robot position to update itself
    ROBOT_INITIATE_MISSION_DELAY: int = Field(default=5)

    # ID for the mission definition we amend to create a full mission
    MISSION_ID: int = Field(default=3)

    RECORD_ROSBAGS: bool = Field(default=False)

    # Number of joints on the robot.
    # Note that we currently support 2 or 4 joints
    ROBOT_NUMBER_OF_JOINTS: int = Field(default=2)

    # The action id of the dock task to be used
    WALLBOX_DOCK_ACTION_ID: int = Field(default=15234)

    # The name of the wallbox
    WALLBOX_NAME: str = Field(default="wallbox_aasgard_grosglockner")

    ####################################################################################
    # For 2 degrees of freedom

    # Joint angle for joint 1 when the arm is at a zero yaw angle
    TWO_DOF_JOINT_1_AT_ZERO_YAW: float = Field(default=1.34)

    # Joint angle for joint 2 when the arm is in a vertical position
    TWO_DOF_JOINT_2_AT_VERTICAL_ARM: float = Field(default=1.8)

    # Position of arm_base_joint relative to robot_frame
    TWO_DOF_ARM_BASE_X: float = Field(default=0.0901)
    TWO_DOF_ARM_BASE_Y: float = Field(default=0.038)
    TWO_DOF_ARM_BASE_Z: float = Field(default=0.3347)

    # Length of robot arm links
    TWO_DOF_ARM_LINK_1: float = Field(default=0.1)
    TWO_DOF_ARM_LINK_2: float = Field(default=0.50)

    # Physical constraints for the joints of the robot arm
    TWO_DOF_J1_LOWER_CONSTRAINT: float = Field(default=0.022)
    TWO_DOF_J1_UPPER_CONSTRAINT: float = Field(default=5.62)

    TWO_DOF_J2_LOWER_CONSTRAINT: float = Field(default=0.022)
    TWO_DOF_J2_UPPER_CONSTRAINT: float = Field(default=3.348)

    ####################################################################################
    # For 4 degrees of freedom

    # Joint angle for joint 1 when the 1st link is oriented straight ahead on the robot
    # pointing in the x-axis of the robot
    FOUR_DOF_JOINT_1_AT_ZERO_POSITION: float = Field(default=2.01)

    # Joint angle for joint 2 when the 2nd link is in a horizontal position and
    # pointing in the direction parallel to the xy-plane
    FOUR_DOF_JOINT_2_AT_ZERO_POSITION: float = Field(default=0.22)

    # Joint angle for joint 3 when the 3rd link is
    # pointing directly opposite of the 2nd link
    FOUR_DOF_JOINT_3_AT_ZERO_POSITION: float = Field(default=0.22)

    # Joint angle for joint 4 when the 4rd link is
    # pointing 90 degrees up from the 3rd link (camera facing forwards)
    FOUR_DOF_JOINT_4_AT_ZERO_POSITION: float = Field(default=0.11)

    # Position of arm_base_joint relative to robot_frame
    FOUR_DOF_ARM_BASE_X: float = Field(default=0.176)
    FOUR_DOF_ARM_BASE_Y: float = Field(default=0.037)
    FOUR_DOF_ARM_BASE_Z: float = Field(default=0.364)

    FOUR_DOF_ARM_LINK_1: float = Field(default=0.1)
    FOUR_DOF_ARM_LINK_2: float = Field(default=0.417)
    FOUR_DOF_ARM_LINK_3: float = Field(default=0.611)
    FOUR_DOF_ARM_LINK_4: float = Field(default=0.1)

    FOUR_DOF_J1_LOWER_CONSTRAINT: float = Field(default=0.02)
    FOUR_DOF_J1_UPPER_CONSTRAINT: float = Field(default=5.655)

    FOUR_DOF_J2_LOWER_CONSTRAINT: float = Field(default=0.02)
    FOUR_DOF_J2_UPPER_CONSTRAINT: float = Field(default=3.420843)

    FOUR_DOF_J3_LOWER_CONSTRAINT: float = Field(default=0.018)
    FOUR_DOF_J3_UPPER_CONSTRAINT: float = Field(default=4.79966)

    FOUR_DOF_J4_LOWER_CONSTRAINT: float = Field(default=0.018)
    FOUR_DOF_J4_UPPER_CONSTRAINT: float = Field(default=4.79966)

    ####################################################################################

    # Enable if the format of the thermal video should be delivered as a sequence file
    # If this is False a mp4 will be delivered
    THERMAL_VIDEO_SEQUENCE_FORMAT: bool = Field(default=False)

    ACOUSTIC_SENSITIVITY: Literal["low", "medium", "high"] = Field(default="high")

    # File formats accepted for the different inspection steps
    # Comma separated string with the supported file formats
    FILE_FORMAT_IMAGE: list[str] = Field(default=["jpeg", "png"])
    FILE_FORMAT_THERMAL_IMAGE: list[str] = Field(default=["fff"])
    FILE_FORMAT_VIDEO: list[str] = Field(default=["mp4"])

    # Video is re-encoded if this is set to true due to some browsers requiring
    # a constant frame rate. Set to False to upload the original bytes.
    NORMALIZE_VIDEO_TO_CONSTANT_FRAME_RATE: bool = Field(default=True)

    # Target constant frame rate (fps) used when normalizing videos.
    VIDEO_TARGET_FRAME_RATE: int = Field(default=25)
    FILE_FORMAT_THERMAL_VIDEO: list[str] = Field(default=["mp4", "seq"])
    FILE_FORMAT_AUDIO: list[str] = Field(default=["wav", "mp4"])

    # Telemetry publisher intervals
    # Determines how often the package collects and sends updates for each telemetry
    # category
    POSE_PUBLISH_INTERVAL: int = Field(default=5)
    BATTERY_PUBLISH_INTERVAL: int = Field(default=5)
    PRESSURE_PUBLISH_INTERVAL: int = Field(default=5)
    OBSTACLE_PUBLISH_INTERVAL: int = Field(default=10)

    # Max age of the cached GET /sensor/all snapshot before a read refreshes it.
    # Also used as the request timeout for the refresh.
    SENSOR_REFRESH_INTERVAL: int = Field(default=3)

    model_config = SettingsConfigDict(
        env_prefix="TAUROB_",
        env_file_encoding="utf-8",
        case_sensitive=True,
        extra="ignore",
    )


settings = Settings()
