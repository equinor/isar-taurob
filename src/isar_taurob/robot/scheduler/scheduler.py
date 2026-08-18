import logging

from robot_interface.models.exceptions.robot_exceptions import (
    RobotInfeasibleTaskException,
    RobotTaskStatusException,
)
from robot_interface.models.mission.mission import MissionStatus
from robot_interface.models.mission.task import (
    RecordAudio,
    TakeImage,
    TakeThermalImage,
    TakeThermalVideo,
    TakeVideo,
    Task,
)

from isar_taurob.config import settings
from isar_taurob.robot.api.api import API
from isar_taurob.robot.api.measurement_metadata import (
    MeasurementMetadata,
    ResultFormat,
    ResultType,
    SensorName,
)
from isar_taurob.robot.api.mission_status import map_taurob_status_to_mission_status
from isar_taurob.robot.api.task_status import map_taurob_status_to_task_status
from isar_taurob.robot.map import MapHandler
from isar_taurob.robot.taurob_model.arm import TaurobArm
from isar_taurob.robot.taurob_model.joints import Joints
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry


class Scheduler:
    def __init__(
        self, robot_api: API, telemetry: TaurobTelemetry, map_handler: MapHandler
    ):
        self.logger = logging.getLogger(settings.LOGGER_NAME)
        self.telemetry: TaurobTelemetry = telemetry
        self.robot_api: API = robot_api
        self.map_handler: MapHandler = map_handler
        self.taurob_arm: TaurobArm = TaurobArm(settings.ROBOT_NUMBER_OF_JOINTS)
        self.joints: Joints = Joints(settings.ROBOT_NUMBER_OF_JOINTS)

    def get_task_status(self, instance_id: int, action_id: int):
        body = self.robot_api.get_mission_instance_details(instance_id=instance_id)
        # TODO: Add exception here for failed 'start', aka localization
        # If ISAR doesn't have task for localization and ask for it. Localization will never be checked

        try:
            task_status = next(
                map_taurob_status_to_task_status(status=step["status"])
                for step in body
                if (
                    step["mission_step"] is not None
                    and step["mission_step"]["step_id"] == action_id
                )
            )
        except StopIteration:
            self.logger.warning(
                f"No task with action_id {action_id} found in mission with instance_id {instance_id}"
            )
            raise RobotTaskStatusException(
                error_description=f"No task with action_id {action_id} found in mission with instance_id {instance_id}"
            )

        return task_status

    def get_mission_status(self, instance_id: int):
        body = self.robot_api.get_mission_instance_details(instance_id=instance_id)

        status = map_taurob_status_to_mission_status(body[0]["status"])

        ## Workaround to handle that mission status sometimes becomes successful even if tasks have failed
        if status == MissionStatus.Successful:
            for step in body:
                step_status = map_taurob_status_to_task_status(step["status"])
                if (
                    step_status == MissionStatus.Failed
                    and step["mission_step"] is not None
                ):
                    self.logger.warning(
                        f"Task with action_id {step['mission_step']['step_id']} failed"
                    )
                    if step["mission_step"]["step_type"] == "dock":
                        return MissionStatus.Failed
                    return MissionStatus.PartiallySuccessful
        ## Workaround end

        return map_taurob_status_to_mission_status(status=body[0]["status"])

    def stop(self) -> None:
        self.robot_api.delete_all_mission_instances()

    def initiate_mission(self, action_ids: list[int]) -> int:
        current_mission_instance_id: int = self.robot_api.schedule(
            mission_id=settings.MISSION_ID,
            step_list={
                "steps": action_ids,
                "options": {
                    "flipper_before_measurement": True,
                    "record_rosbag": settings.RECORD_ROSBAGS,
                    "light": False,
                    "keep_localization_on": True,
                },
            },
        )
        self.logger.debug(
            f"Initiated inspection step with mission instance id: {current_mission_instance_id}"
        )
        return current_mission_instance_id

    def get_measurement_metadata(self, task: Task) -> MeasurementMetadata:
        if isinstance(task, TakeImage):
            return MeasurementMetadata(
                sensor_name=SensorName.Visual,
                result_type=ResultType.Image,
                result_format=ResultFormat.Image,
            )

        elif isinstance(task, TakeThermalImage):
            return MeasurementMetadata(
                sensor_name=SensorName.Thermal,
                result_type=ResultType.Image,
                result_format=ResultFormat.Image,
            )

        elif isinstance(task, TakeVideo):
            return MeasurementMetadata(
                sensor_name=SensorName.Thermal,
                result_type=ResultType.Video,
                result_format=ResultFormat.Video,
                duration=task.duration,
            )

        elif (
            isinstance(task, TakeThermalVideo)
            and settings.THERMAL_VIDEO_SEQUENCE_FORMAT
        ):
            return MeasurementMetadata(
                sensor_name=SensorName.Thermal,
                result_type=ResultType.ThermalVideo,
                result_format=ResultFormat.ThermalVideo,
                duration=task.duration,
            )

        # This is for collecting the thermal video as mp4
        elif isinstance(task, TakeThermalVideo):
            return MeasurementMetadata(
                sensor_name=SensorName.Thermal,
                result_type=ResultType.Video,
                result_format=ResultFormat.Video,
                duration=task.duration,
            )

        elif isinstance(task, RecordAudio):
            return MeasurementMetadata(
                sensor_name=SensorName.Audio,
                result_type=ResultType.Audio,
                result_format=ResultFormat.Audio,
                duration=task.duration,
            )

        else:
            error_description: str = (
                f"An inspection task with unsupported "
                f"type '{task.__class__.__name__}' was provided when selecting metadata type"
            )
            self.logger.error(error_description)
            raise RobotInfeasibleTaskException(error_description=error_description)
