import logging
import time
from collections.abc import Callable
from datetime import UTC, datetime
from functools import partial
from logging import Logger
from threading import Thread

import pytz
from alitra import Pose
from robot_interface.mission_recording import MissionRecorder
from robot_interface.models.exceptions.robot_exceptions import (
    RobotAlreadyHomeException,
    RobotAPIException,
    RobotCommunicationException,
    RobotDockingStatusMismatchException,
    RobotInfeasibleMissionException,
    RobotMissionStatusException,
    RobotNoMissionRunningException,
    RobotRetrieveInspectionException,
    RobotTaskStatusException,
)
from robot_interface.models.inspection.inspection import Inspection, InspectionMetadata
from robot_interface.models.mission.mission import Mission
from robot_interface.models.mission.status import MissionStatus, RobotStatus
from robot_interface.models.mission.task import (
    TASKS,
    InspectionTask,
    TakeAcousticMeasurement,
    TakeXam8000GasMeasurement,
    TaskStatus,
)
from robot_interface.models.robots.battery_state import BatteryState
from robot_interface.models.robots.media import MediaConfig
from robot_interface.recording_publisher import RecordingPublisher
from robot_interface.robot_interface import RobotInterface
from robot_interface.telemetry.mqtt_client import TelemetryParameters
from robot_interface.telemetry.payloads import (
    TelemetryBatteryPayload,
    TelemetryObstacleStatusPayload,
    TelemetryPosePayload,
    TelemetryPressurePayload,
)

from isar_taurob.config import settings
from isar_taurob.open_telemetry.metrics_handler import MetricsHandler
from isar_taurob.robot.api.api import API, ActionType
from isar_taurob.robot.api.measurement_metadata import MeasurementMetadata
from isar_taurob.robot.api.mission_status import is_mission_completed
from isar_taurob.robot.api.request_handler import RequestHandler
from isar_taurob.robot.map import MapHandler
from isar_taurob.robot.media import create_ome_media_config
from isar_taurob.robot.recording.mission_recorder import TaurobMissionRecorder
from isar_taurob.robot.recording.ome_capture import capture_ome
from isar_taurob.robot.scheduler.scheduler import Scheduler
from isar_taurob.robot.storage.storage import Storage
from isar_taurob.robot.taurob_model.arm import TaurobArm
from isar_taurob.robot.taurob_model.inspection_tuple import ActionTaskTuple
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry


class Robot(RobotInterface):
    def get_mission_recorder(
        self, publisher: RecordingPublisher | None = None
    ) -> MissionRecorder | None:
        if publisher is None:
            raise RuntimeError("Recording requires a platform publisher")
        return TaurobMissionRecorder(
            partial(capture_ome, self.generate_media_config),
            publisher,
            directory=settings.RECORDING_DATA_DIRECTORY,
        )

    def __init__(self, robot_name: str, isar_id: str) -> None:
        super().__init__(robot_name=robot_name, isar_id=isar_id)

        self.task_id_to_completion_pose: dict[str, TaurobPose] = {}
        self.task_id_to_completion_time_stamp: dict = {}
        self.instance_id: int | None = None
        self.last_mission_id: str | None = None
        self.is_return_home_mission: bool = False

        self.map_handler = MapHandler()
        self.robot_api: API = API(request_handler=RequestHandler())
        self.taurob_arm: TaurobArm = TaurobArm(settings.ROBOT_NUMBER_OF_JOINTS)

        self.telemetry: TaurobTelemetry = TaurobTelemetry(self.robot_api)
        self.scheduler: Scheduler = Scheduler(
            robot_api=self.robot_api,
            telemetry=self.telemetry,
            map_handler=self.map_handler,
        )
        self.storage: Storage = Storage(
            robot_api=self.robot_api,
            robot_telemetry=self.telemetry,
        )

        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)

        self.paused_mission_instance_id: int | None = None
        self.action_task_map: dict[str, ActionTaskTuple] = {}

        self.metrics_handler: MetricsHandler = MetricsHandler(
            telemetry=self.telemetry, robot_name=robot_name, isar_id=isar_id
        )

        self.logger.info("ISAR taurob initialized")

    def _create_dock_action(self) -> int:
        # Returns step used for docking and undocking
        return settings.WALLBOX_DOCK_ACTION_ID

    def _create_drive_to_task(self, task_pose: Pose, task_id: str) -> int:
        map_name: str = self.map_handler.select_map_by_asset_position(
            task_pose.position
        ).name
        taurob_pose = self.map_handler.transform_pose_to_robot(pose=task_pose)

        return self.robot_api.create_action(
            task_id, ActionType.driveTo, taurob_pose, map_name
        )

    def _create_inspection_task(self, task: InspectionTask):
        # Gas measurements are continously available as a sensor call, and do therefor only require a driveTo action
        if isinstance(task, TakeXam8000GasMeasurement):
            return self._create_drive_to_task(
                task_pose=task.robot_pose, task_id=task.id
            )

        robot_pose = self.map_handler.transform_pose_to_robot(pose=task.robot_pose)
        try:
            robot_target_position = self.map_handler.transform_position_to_robot(
                position=task.target
            )
            joints = self.taurob_arm.solve_inverse_kinematics(
                robot_pose=robot_pose,
                target=robot_target_position,
            )

            if len(joints.constrainted_joints) > 0:
                for (
                    constrained_joint,
                    joint_value,
                ) in joints.constrainted_joints:
                    self.logger.warning(
                        f"Joint {constrained_joint} is constrained to value {joint_value} "
                        f"for tag_id = {task.tag_id}"
                    )
        except AttributeError:
            self.logger.warning(
                f"No inspection target found for task {task.id} of type {task.__class__.__name__}. Setting joints to None."
            )
            joints = None

        measurement_metadata: MeasurementMetadata = (
            self.scheduler.get_measurement_metadata(task=task)
        )

        map_name: str = self.map_handler.select_map_by_asset_position(
            task.robot_pose.position
        ).name
        return self.robot_api.create_action(
            task.id,
            ActionType.measure,
            robot_pose,
            map_name,
            joints=joints,
            metadata=measurement_metadata,
        )

    def _get_is_charging(self) -> bool:
        time.sleep(2)  # Ensure ample time for updating charging value in cloud
        self.telemetry.invalidate_sensor_cache()
        _, batteryState = self.telemetry.get_robot_battery()
        return_value = batteryState == BatteryState.Charging
        return return_value

    def _get_is_docked(self) -> bool:
        if self.telemetry.get_robot_state() != "docked":
            return False

        return True

    def _get_is_localization_running(self) -> bool:
        return self.telemetry.get_robot_is_localization_running()

    def _create_actions(
        self,
        tasks: list[TASKS],
    ) -> list[int]:
        action_ids: list[int] = []
        self.action_task_map = {}

        if self._get_is_docked():
            action_ids.append(self._create_dock_action())

        for task in tasks:
            action_id = self._create_inspection_task(task)

            self.action_task_map[task.id] = ActionTaskTuple(
                mission_instance_id=None,  # Unknown until after scheduling
                isar_task_id=task.id,
                taurob_action_id=action_id,
            )
            action_ids.append(action_id)

        return action_ids

    def _is_robot_home(self) -> bool:
        is_charging = self._get_is_charging()
        is_docked = self._get_is_docked()

        if is_docked != is_charging:
            raise RobotDockingStatusMismatchException(
                "Robot docking status and charging status are inconsistent."
            )
        return is_charging

    def initiate_mission(self, mission: Mission) -> None:
        time.sleep(settings.ROBOT_INITIATE_MISSION_DELAY)

        self.logger.info(
            f"Starting initiate mission for mission with id {mission.id} and resetting previous instance_id"
        )
        self.instance_id = None

        is_remote_stop: bool = self.telemetry.get_robot_remote_stop()
        if is_remote_stop:
            raise RobotInfeasibleMissionException(
                "Robot is in remote stop mode. Cannot initiate mission."
            )

        if not self._get_is_localization_running() and not self._get_is_docked():
            raise RobotInfeasibleMissionException(
                "Robot localization is not running, nor is the robot docked. Cannot initiate mission."
            )

        self.logger.info(f"Received mission with ID: {mission.id}")
        action_ids: list[int]

        action_ids = self._create_actions(
            mission.tasks,
        )

        self.robot_api.amend_mission(mission.name, action_ids)

        self.is_return_home_mission = False
        self.instance_id = self.scheduler.initiate_mission(action_ids)
        self.last_mission_id = mission.id

        self.logger.info(f"Starting mission with instance ID: {self.instance_id}")

        self._assign_instance_id_to_action_task_map(instance_id=self.instance_id)

    def _assign_instance_id_to_action_task_map(self, instance_id: int) -> None:
        for action_task_tuple in self.action_task_map.values():
            action_task_tuple.mission_instance_id = instance_id

    def initiate_return_home(self, mission_id: str) -> None:
        time.sleep(settings.ROBOT_INITIATE_MISSION_DELAY)

        self.logger.info(
            f"Starting initiate mission for return home with id {mission_id} and resetting previous instance_id"
        )
        self.instance_id = None

        if self._is_robot_home():
            raise RobotAlreadyHomeException(
                "Robot is already home and charging. Disregarding initiate of return to home mission."
            )

        is_remote_stop: bool = self.telemetry.get_robot_remote_stop()
        if is_remote_stop:
            raise RobotInfeasibleMissionException(
                "Robot is in remote stop mode. Cannot initiate mission."
            )

        if not self._get_is_localization_running():
            raise RobotInfeasibleMissionException(
                "Robot localization is not running. Cannot initiate return home mission."
            )

        self.logger.info("Received return home mission request")
        self.is_return_home_mission = True
        self.instance_id = self.robot_api.schedule_return_home()
        self.last_mission_id = mission_id

        self.logger.info(f"Starting mission with instance ID: {self.instance_id}")

    def _get_task_status(self, instance_id, task_id) -> TaskStatus:
        task_status: TaskStatus
        task_status = self.scheduler.get_task_status(
            instance_id=instance_id,
            action_id=self.action_task_map[task_id].taurob_action_id,
        )

        return task_status

    def _evaluate_if_task_paused(
        self, paused_mission_instance_id: int | None, task_status: TaskStatus
    ) -> bool:
        # To handle task status when mission is changing from monitor to paused
        paused_mission_instance_id = (
            paused_mission_instance_id
            if paused_mission_instance_id
            else self.paused_mission_instance_id
        )
        if self.instance_id != paused_mission_instance_id:
            return False

        if task_status == TaskStatus.Cancelled:
            return True

        return False

    def task_status(self, task_id: str) -> TaskStatus:
        task_status: TaskStatus

        if self.instance_id is None:
            error_message = "Error fetching task status. Instance ID is None."
            self.logger.error(error_message)
            raise RobotTaskStatusException(error_description=error_message)

        # To handle task status when mission is changing from paused to resume
        paused_mission_instance_id: int | None = self.paused_mission_instance_id

        task_status = self._get_task_status(self.instance_id, task_id)

        if self._evaluate_if_task_paused(paused_mission_instance_id, task_status):
            return TaskStatus.Paused

        if (
            task_status == TaskStatus.Successful
            and task_id not in self.task_id_to_completion_pose
        ):
            self.telemetry.invalidate_sensor_cache()
            self.task_id_to_completion_pose[task_id] = self.telemetry.get_robot_pose()
            self.task_id_to_completion_time_stamp[task_id] = datetime.now(UTC)

        if task_status == TaskStatus.Failed:
            try:
                failure_reason, timestamp = self.telemetry.get_mission_failure_reason()
                self.logger.error(
                    f"Task with ID {task_id} failed with reason: {failure_reason} at {timestamp}"
                )
            except RobotAPIException:
                self.logger.error("Failed to retrieve failure reason")

        return task_status

    def _evaluate_if_mission_paused(
        self,
        instance_id: int,
        paused_mission_instance_id: int | None,
        mission_status: MissionStatus,
    ) -> bool:
        # To handle mission status when mission is changing from monitor to paused
        paused_mission_instance_id = (
            paused_mission_instance_id
            if paused_mission_instance_id
            else self.paused_mission_instance_id
        )
        if instance_id != paused_mission_instance_id:
            return False

        if mission_status == MissionStatus.Cancelled:
            return True

        return False

    def mission_status(self, mission_id: str) -> MissionStatus:
        instance_id: int | None = self.instance_id
        if mission_id != self.last_mission_id or instance_id is None:
            raise RobotMissionStatusException(
                error_description=f"Mission with id {mission_id} is not known or not running"
            )

        # To handle mission_status when mission is changing from paused to resume
        paused_mission_instance_id = self.paused_mission_instance_id

        mission_status = self.scheduler.get_mission_status(instance_id)

        if self._evaluate_if_mission_paused(
            instance_id, paused_mission_instance_id, mission_status
        ):
            return MissionStatus.Paused

        if is_mission_completed(mission_status):
            self.logger.info(f"Mission with instance ID {instance_id} is completed.")
            self.instance_id = None
            self.last_mission_id = None

        return mission_status

    def stop(self) -> None:
        if self.instance_id is None:
            # Handle that paused missions are already cancelled so stop is redundant
            if self.paused_mission_instance_id is not None:
                self.paused_mission_instance_id = None
                self.logger.info(
                    "Received stop command while a mission was paused. Cleared paused mission."
                )
                return

            self.logger.info(
                "No known mission instance to stop. Will check in the cloud for any scheduled missions."
            )
            instance_ids = self.robot_api.get_scheduled_missions()
            if len(instance_ids) == 0:
                self.logger.warning(
                    "No scheduled missions found in the cloud when trying to stop mission."
                )
                raise RobotNoMissionRunningException(
                    error_description="Attempted to stop non-existent mission"
                )
            self.robot_api.delete_mission_instance(instance_ids[0])
        else:
            self.robot_api.delete_mission_instance(self.instance_id)
        time.sleep(5)  # Ensure ample time for robot position to update in the cloud
        self.telemetry.invalidate_sensor_cache()

    def get_inspections(self, task: InspectionTask) -> list[Inspection]:
        try:
            inspection_tuple: ActionTaskTuple = self.action_task_map[task.id]
        except KeyError:
            raise RobotRetrieveInspectionException(
                f"Failed to find task with ID {task.id}. No inspection will be retrieved."
            )
        if inspection_tuple.mission_instance_id is None:
            raise RobotRetrieveInspectionException(
                f"Unknown mission instance ID for task with ID {task.id}. No inspection will be retrieved."
            )

        try:
            taurob_pose: TaurobPose = self.task_id_to_completion_pose.pop(task.id)
            time_stamp: datetime = self.task_id_to_completion_time_stamp.pop(task.id)
        except KeyError:
            raise RobotRetrieveInspectionException(
                f"No completion pose/timestamp found for task {task.id}. "
                "No inspection will be retrieved."
            )

        asset_pose: Pose = self.map_handler.transform_pose_to_asset(pose=taurob_pose)

        metadata_type: type[InspectionMetadata] = (
            InspectionMetadata
            if isinstance(task, TakeAcousticMeasurement)
            else task.get_inspection_type().get_metadata_type()
        )

        try:
            target_position = task.target
        except AttributeError:
            self.logger.debug(
                "No inspection target specified, using robot position instead"
            )
            target_position = asset_pose.position

        metadata_kwargs: dict = {
            "start_time": time_stamp,
            "robot_pose": asset_pose,
            "target_position": target_position,
            "file_type": "",
        }
        if hasattr(task, "duration"):
            metadata_kwargs["duration"] = task.duration

        metadata: InspectionMetadata = metadata_type(**metadata_kwargs)
        metadata.tag_id = task.tag_id
        metadata.inspection_description = (
            task.inspection_description
            if task.inspection_description is not None
            else ""
        )

        return self.storage.get_inspections(
            instance_id=inspection_tuple.mission_instance_id,
            inspection_metadata=metadata,
            action_id=inspection_tuple.taurob_action_id,
            task=task,
        )

    def register_inspection_callback(
        self, callback_function: Callable[[Inspection], None]
    ) -> None:
        raise NotImplementedError

    def initialize(self) -> None:
        return None

    def get_telemetry_publishers(self) -> list[TelemetryParameters]:
        return [
            TelemetryParameters(
                name="ISAR Robot Pose Publisher",
                method=lambda: self._get_pose_telemetry(),
                topic="pose",
                interval=settings.POSE_PUBLISH_INTERVAL,
            ),
            TelemetryParameters(
                name="ISAR Robot Battery Publisher",
                method=lambda: self._get_battery_telemetry(),
                topic="battery",
                interval=settings.BATTERY_PUBLISH_INTERVAL,
            ),
            TelemetryParameters(
                name="ISAR Robot Obstacle Publisher",
                method=lambda: self._get_obstacle_status_telemetry(),
                topic="obstacle_detected",
                interval=settings.OBSTACLE_PUBLISH_INTERVAL,
            ),
            TelemetryParameters(
                name="ISAR Robot Pressure Publisher",
                method=lambda: self._get_pressure_telemetry(),
                topic="pressure",
                interval=settings.PRESSURE_PUBLISH_INTERVAL,
            ),
        ]

    def get_utility_threads(self) -> list[Thread]:
        return []

    def _get_pose_telemetry(self) -> str:
        pose_payload = TelemetryPosePayload(
            pose=self._get_robot_pose_in_asset_frame(),
        )
        return pose_payload.model_dump_json()

    def _get_battery_telemetry(self) -> str:
        battery_value_list = self.telemetry.get_robot_battery()
        battery_payload = TelemetryBatteryPayload(
            battery_level=battery_value_list[0],
            battery_state=battery_value_list[1],
        )
        return battery_payload.model_dump_json()

    def _get_obstacle_status_telemetry(self) -> str:
        obstacle_status_payload = TelemetryObstacleStatusPayload(
            obstacle_status=self.telemetry.get_robot_obstacle_status(),
        )
        return obstacle_status_payload.model_dump_json()

    def _get_pressure_telemetry(self) -> str:
        pressure_payload = TelemetryPressurePayload(
            pressure_level=self.telemetry.get_robot_pressure_level(),
        )
        return pressure_payload.model_dump_json()

    def _get_robot_pose_in_asset_frame(self) -> Pose:
        taurob_pose: TaurobPose = self.telemetry.get_robot_pose()
        return self.map_handler.transform_pose_to_asset(taurob_pose)

    def robot_status(self) -> RobotStatus:
        now: datetime = datetime.now().astimezone(tz=pytz.utc)
        time_since_last_update: float = (
            now - self.telemetry.last_update_from_robot
        ).total_seconds()

        if time_since_last_update > settings.ROBOT_REQUESTS_TIMEOUT:
            return RobotStatus.Offline

        if self.telemetry.get_robot_remote_stop():
            return RobotStatus.TeleOperation

        if self._get_is_docked():
            return RobotStatus.Home

        if self.robot_api.get_scheduled_actions(settings.ROBOT_NAME):
            return RobotStatus.Busy
        return RobotStatus.Available

    def pause(self) -> None:
        if not self.instance_id:
            raise RobotNoMissionRunningException(
                error_description="Attempted to pause non-existent mission"
            )
        self.paused_mission_instance_id = self.instance_id
        self.logger.info(
            f"PAUSE: paused_mission_instance_id is {self.paused_mission_instance_id}"
        )
        self.stop()

    def resume(self) -> None:
        self.logger.info("RESUME: Started resume mission")

        if self.paused_mission_instance_id is None:
            self.logger.error(
                "RESUME: Failed to resume as there is no mission to resume"
            )
            raise RobotNoMissionRunningException(
                error_description="Attempted to resume non-existent mission"
            )

        if self.is_return_home_mission:
            self.logger.info("RESUME: Scheduling return home mission.")
            self.instance_id = self.robot_api.schedule_return_home()
            self.paused_mission_instance_id = None
            return

        try:
            body = self.robot_api.get_mission_instance_details(
                instance_id=self.paused_mission_instance_id
            )
        except RobotCommunicationException as e:
            error_description = (
                "Could not retrieve mission details from API when attempting to resume"
            )
            self.logger.error(
                f"RESUME: {error_description}. Error message: {e.error_description}"
            )
            raise RobotCommunicationException(error_description=error_description)

        action_ids_list: list[int] = [
            action["mission_step"]["step_id"]
            for action in body
            if action["status"] == "canceled"
            and action["mission_step"] is not None
            and not (  # Remove undock step
                action["mission_step"]["step_type"] == "dock"
                and action["mission_step"]["step_number"] == 1
            )
        ]

        if self._get_is_docked():
            action_ids_list.insert(0, self._create_dock_action())
            self.logger.info("Adding new initial action.")

        index_of_cancelled_step = len(body) - 1 - len(action_ids_list)
        # -1 accounts for the fact that we have mission options introduced as an action

        if len(action_ids_list) == 0:
            error_description = f"Failed to resume mission with id {self.paused_mission_instance_id} as there are no cancelled actions to resume."
            self.logger.warning(f"RESUME: {error_description}")
            raise RobotAPIException(error_description=error_description)

        if index_of_cancelled_step != 0:
            mission_name = "Resumed mission " + str(self.paused_mission_instance_id)
            self.robot_api.amend_mission(mission_name, action_ids_list)
        try:
            self.instance_id = self.scheduler.initiate_mission(action_ids_list)
            self._assign_instance_id_to_action_task_map(instance_id=self.instance_id)
            self.paused_mission_instance_id = None
            self.logger.info(
                f"RESUME: Successfully resumed mission with new instance ID {self.instance_id}"
            )
        except RobotAPIException as e:
            error_description = (
                f"Failed to resume mission with action ids '{action_ids_list}'"
            )
            self.logger.error(f"RESUME: {error_description}. Error message: {e} ")
            raise RobotAPIException(error_description=error_description)

    def generate_media_config(self) -> MediaConfig | None:
        return create_ome_media_config(
            self.robot_api.get_robot_video_streams(), settings.ROBOT_NAME
        )

    def get_battery_level(self) -> float:
        return self.telemetry.last_battery_measurement
