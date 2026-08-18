import logging
import time
from collections.abc import Callable
from datetime import UTC, datetime
from logging import Logger
from queue import Queue
from threading import Thread

import pytz
from alitra import Pose
from robot_interface.models.exceptions.robot_exceptions import (
    RobotAlreadyHomeException,
    RobotAPIException,
    RobotCommunicationException,
    RobotInfeasibleMissionException,
    RobotMissionStatusException,
    RobotNoMissionRunningException,
    RobotRetrieveInspectionException,
    RobotTaskStatusException,
    RobotUnreliableDockingStatusException,
)
from robot_interface.models.inspection.inspection import Inspection, InspectionMetadata
from robot_interface.models.mission.mission import Mission
from robot_interface.models.mission.status import MissionStatus, RobotStatus
from robot_interface.models.mission.task import (
    InspectionTask,
    ReturnToHome,
    Task,
    TaskStatus,
)
from robot_interface.models.robots.battery_state import BatteryState
from robot_interface.models.robots.media import MediaConfig
from robot_interface.robot_interface import RobotInterface
from robot_interface.telemetry.mqtt_client import MqttTelemetryPublisher
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
from isar_taurob.robot.scheduler.scheduler import Scheduler
from isar_taurob.robot.storage.storage import Storage
from isar_taurob.robot.taurob_model.arm import TaurobArm
from isar_taurob.robot.taurob_model.inspection_tuple import ActionTaskTuple
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose
from isar_taurob.robot.telemetry.taurob_telemetry import TaurobTelemetry


class Robot(RobotInterface):
    def __init__(self, robot_name: str, isar_id: str) -> None:
        super().__init__(robot_name=robot_name, isar_id=isar_id)

        self.task_id_to_completion_pose: dict[str, TaurobPose] = {}
        self.task_id_to_completion_time_stamp: dict = {}
        self.instance_id: int | None = None
        self.last_mission_id: str | None = None

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
        )

        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)

        self.paused_mission_instance_id: int | None = None
        self.action_task_map: dict[str, ActionTaskTuple] = {}
        self.confirmed_docked_since_last_mission: bool = False

        self.metrics_handler: MetricsHandler = MetricsHandler(
            telemetry=self.telemetry, robot_name=robot_name, isar_id=isar_id
        )

        self.logger.info("ISAR taurob initialized")

    def _create_localize_action(self, task_pose: Pose, task_id: str) -> int:
        map_name: str = self.map_handler.select_map_by_asset_position(
            task_pose.position
        ).name
        taurob_pose = self.map_handler.transform_pose_to_robot(pose=task_pose)

        return self.robot_api.create_action(
            task_id, ActionType.localize, taurob_pose, map_name
        )

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

    def _create_localization_action_at_current_pose(self) -> int:
        taurob_pose = self.telemetry.get_robot_pose()
        current_robot_pose = self.map_handler.transform_pose_to_asset(pose=taurob_pose)
        self.logger.info(
            f"Using current robot pose '{current_robot_pose}' as localization pose."
        )
        return self._create_localize_action(current_robot_pose, "localize")

    def _get_is_charging(self) -> bool:
        time.sleep(2)  # Ensure updated charging value in cloud
        _, batteryState = self.telemetry.get_robot_battery()
        return_value = batteryState == BatteryState.Charging
        return return_value

    def _get_is_docked(self) -> bool:
        robot_state: dict = self.robot_api.get_sensor_value(sensor_name="robot.state")

        if robot_state["value"] != "docked":
            return False

        self.confirmed_docked_since_last_mission = True
        return True

    def _create_initial_action(self) -> int:
        if self._get_is_docked() or self.confirmed_docked_since_last_mission:
            return self._create_dock_action()
        else:
            return self._create_localization_action_at_current_pose()

    def _create_actions(
        self,
        tasks: list[Task],
    ) -> list[int]:
        action_ids: list[int] = []
        self.action_task_map = {}

        action_id = self._create_initial_action()
        action_ids.append(action_id)

        for task in tasks:
            type: str
            if isinstance(task, ReturnToHome):
                # If robot should dock, the return home step will be replaced by a dock step
                if task.id == tasks[-1].id:
                    action_id = self._create_dock_action()
                    self.logger.info(
                        "Received return to home task was added as a dock action"
                    )
                type = "return_to_home"
            elif isinstance(task, InspectionTask):
                action_id = self._create_inspection_task(task)
                type = "inspection_type"
            else:
                raise RobotInfeasibleMissionException(
                    f"Task {task} is of unsupported type"
                )

            self.action_task_map[task.id] = ActionTaskTuple(
                mission_instance_id=None,  # Unknown until after scheduling
                isar_task_id=task.id,
                taurob_action_id=action_id,
                joints=None,  # TODO: Get correct value for joints
                type=type,
            )
            action_ids.append(action_id)

        return action_ids

    def _is_return_to_home_mission(self, mission: Mission) -> bool:
        return len(mission.tasks) == 1 and isinstance(mission.tasks[0], ReturnToHome)

    def _is_robot_home(self) -> bool:
        is_charging = self._get_is_charging()
        is_docked = self._get_is_docked()

        if is_docked != is_charging:
            raise RobotUnreliableDockingStatusException(
                "Robot docking status and charging status are inconsistent."
            )
        return is_charging

    def initiate_mission(self, mission: Mission) -> None:
        time.sleep(settings.ROBOT_INITIATE_MISSION_DELAY)

        self.logger.info(
            f"Starting initiate mission for mission with id {mission.id} and resetting previous instance_id"
        )
        self.instance_id = None

        is_robot_home: bool = self._is_robot_home()

        if is_robot_home and self._is_return_to_home_mission(mission):
            raise RobotAlreadyHomeException(
                "Robot is already home and charging. Disregarding initiate of return to home mission."
            )

        is_remote_stop: bool = self.telemetry.get_robot_remote_stop()
        if is_remote_stop:
            raise RobotInfeasibleMissionException(
                "Robot is in remote stop mode. Cannot initiate mission."
            )

        self.logger.info(f"Received mission with ID: {mission.id}")
        action_ids: list[int]

        action_ids = self._create_actions(
            mission.tasks,
        )

        self.robot_api.amend_mission(mission.name, action_ids)

        self.instance_id = self.scheduler.initiate_mission(action_ids)
        self.last_mission_id = mission.id

        self.logger.info(f"Starting mission with instance ID: {self.instance_id}")

        self._assign_instance_id_to_action_task_map(instance_id=self.instance_id)

    def _assign_instance_id_to_action_task_map(self, instance_id: int) -> None:
        for action_task_tuple in self.action_task_map.values():
            action_task_tuple.mission_instance_id = instance_id

    def _is_last_task_in_mission(self, task_id):
        return list(self.action_task_map.keys())[-1] == task_id

    def _get_task_status(self, instance_id, task_id) -> TaskStatus:
        task_status: TaskStatus
        task_status = self.scheduler.get_task_status(
            instance_id=instance_id,
            action_id=self.action_task_map[task_id].taurob_action_id,
        )

        if self.confirmed_docked_since_last_mission:
            # If any inspection/driveTo task has started, the robot should have a successful localization and is no longer home
            if (
                task_status == TaskStatus.InProgress
                or task_status == TaskStatus.Successful
            ):
                self.confirmed_docked_since_last_mission = False
                self.logger.info("Robot is no longer home.")

        if (
            self.action_task_map[task_id].type == "return_to_home"
            and task_status == TaskStatus.Successful
        ):
            self.confirmed_docked_since_last_mission = True
            self.logger.info("Robot has successfully returned home")

        return task_status

    def task_status(self, task_id: str) -> TaskStatus:
        task_status: TaskStatus

        if self.instance_id is None:
            error_message = "Error fetching task status. Instance ID is None."
            self.logger.error(error_message)
            raise RobotTaskStatusException(error_description=error_message)

        # To handle task status when mission is changing from paused to resume
        paused_mission_instance_id = self.paused_mission_instance_id

        task_status = self._get_task_status(self.instance_id, task_id)

        ## Workaround to avoid cancelled status on paused missions
        # To handle task status when mission is changing from monitor to paused
        paused_mission_instance_id = (
            paused_mission_instance_id
            if paused_mission_instance_id
            else self.paused_mission_instance_id
        )
        if (
            self.instance_id == paused_mission_instance_id
            and task_status == TaskStatus.Cancelled
        ):
            self.logger.debug(
                "Task status is cancelled, but mission is paused. Reporting as paused instead."
            )
            task_status = TaskStatus.Paused
        ## End workaround

        if (
            task_status == TaskStatus.Successful
            and task_id not in self.task_id_to_completion_pose
        ):
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

    def mission_status(self, mission_id: str) -> MissionStatus:
        if mission_id != self.last_mission_id or self.instance_id is None:
            raise RobotMissionStatusException(
                error_description=f"Mission with id {mission_id} is not known or not running"
            )

        # To handle mission_status when mission is changing from paused to resume
        paused_mission_instance_id = self.paused_mission_instance_id
        instance_id = self.instance_id

        mission_status = self.scheduler.get_mission_status(instance_id)

        ## Workaround to avoid cancelled status on paused missions
        # To handle mission status when mission is changing from monitor to paused
        paused_mission_instance_id = (
            paused_mission_instance_id
            if paused_mission_instance_id
            else self.paused_mission_instance_id
        )
        if (
            instance_id == paused_mission_instance_id
            and mission_status == MissionStatus.Cancelled
        ):
            self.logger.debug(
                "Mission status is cancelled, but mission is paused. Reporting as paused instead."
            )
            mission_status = MissionStatus.Paused
        ## End workaround

        if is_mission_completed(mission_status):
            self.logger.info(f"Mission with instance ID {instance_id} is completed.")
            self.instance_id = None
            self.last_mission_id = None

        return mission_status

    def stop(self) -> None:
        if self.instance_id is None:
            ## Workaround to handle that paused missions are cancelled and not really paused
            if self.paused_mission_instance_id is not None:
                self.paused_mission_instance_id = None
                self.logger.info(
                    "Received stop command while a mission was paused. Cleared paused mission."
                )
                return
            ## Workaround end

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
        time.sleep(5)  # Needed to wait for robot position to update in the cloud

    def get_inspection(self, task: InspectionTask) -> Inspection:
        try:
            inspection_tuple: ActionTaskTuple = self.action_task_map[task.id]
        except KeyError:
            raise RobotRetrieveInspectionException(
                f"Failed to find task with ID {task.id}. No inspection will be retrieved."
            )
        if inspection_tuple.type != "inspection_type":
            raise RobotRetrieveInspectionException(
                f"Task with ID {task.id} is not of type inspection so no inspection will be retrieved."
            )
        elif inspection_tuple.mission_instance_id is None:
            raise RobotRetrieveInspectionException(
                f"Unknown mission instance ID for task with ID {task.id}. No inspection will be retrieved."
            )

        taurob_pose: TaurobPose = self.task_id_to_completion_pose[task.id]
        asset_pose: Pose = self.map_handler.transform_pose_to_asset(pose=taurob_pose)

        time_stamp: datetime = self.task_id_to_completion_time_stamp[task.id]

        metadata_type: type[InspectionMetadata] = (
            task.get_inspection_type().get_metadata_type()
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

        return self.storage.get_inspection(
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

    def get_telemetry_publishers(
        self, queue: Queue, isar_id, robot_name: str
    ) -> list[Thread]:
        publisher_threads: list[Thread] = []

        pose_publisher: MqttTelemetryPublisher = MqttTelemetryPublisher(
            mqtt_queue=queue,
            telemetry_method=self._get_pose_telemetry,
            topic=f"isar/{isar_id}/pose",
            interval=settings.POSE_PUBLISH_INTERVAL,
            retain=False,
        )
        pose_thread: Thread = Thread(
            target=pose_publisher.run,
            args=[isar_id, robot_name],
            name="ISAR Robot Pose Publisher",
            daemon=True,
        )
        publisher_threads.append(pose_thread)

        battery_publisher: MqttTelemetryPublisher = MqttTelemetryPublisher(
            mqtt_queue=queue,
            telemetry_method=self._get_battery_telemetry,
            topic=f"isar/{isar_id}/battery",
            interval=settings.BATTERY_PUBLISH_INTERVAL,
            retain=False,
        )
        battery_thread: Thread = Thread(
            target=battery_publisher.run,
            args=[isar_id, robot_name],
            name="ISAR Robot Battery Publisher",
            daemon=True,
        )
        publisher_threads.append(battery_thread)

        obstacle_status_publisher: MqttTelemetryPublisher = MqttTelemetryPublisher(
            mqtt_queue=queue,
            telemetry_method=self._get_obstacle_status_telemetry,
            topic=f"isar/{isar_id}/obstacle_detected",
            interval=settings.OBSTACLE_PUBLISH_INTERVAL,
            retain=False,
        )
        obstacle_status_thread: Thread = Thread(
            target=obstacle_status_publisher.run,
            args=[isar_id, robot_name],
            name="ISAR Robot Obstacle Status Publisher",
            daemon=True,
        )
        publisher_threads.append(obstacle_status_thread)

        pressure_publisher: MqttTelemetryPublisher = MqttTelemetryPublisher(
            mqtt_queue=queue,
            telemetry_method=self._get_pressure_telemetry,
            topic=f"isar/{isar_id}/pressure",
            interval=settings.PRESSURE_PUBLISH_INTERVAL,
            retain=False,
        )
        pressure_thread: Thread = Thread(
            target=pressure_publisher.run,
            args=[isar_id, robot_name],
            name="ISAR Robot Pressure Publisher",
            daemon=True,
        )
        publisher_threads.append(pressure_thread)

        temperature_update_thread: Thread = Thread(
            target=self._run_temperature_update_loop,
            name="ISAR Robot Temperature Updater",
            daemon=True,
        )
        publisher_threads.append(temperature_update_thread)

        cpu_load_update_thread: Thread = Thread(
            target=self._run_cpu_load_update_loop,
            name="ISAR Robot CPU Load Updater",
            daemon=True,
        )
        publisher_threads.append(cpu_load_update_thread)
        return publisher_threads

    def _run_temperature_update_loop(self) -> None:
        while True:
            self.telemetry.update_joint_temperatures()
            time.sleep(settings.TEMPERATURE_PUBLISH_INTERVAL)

    def _run_cpu_load_update_loop(self) -> None:
        while True:
            self.telemetry.update_cpu_load()
            time.sleep(settings.CPU_LOAD_PUBLISH_INTERVAL)

    def _get_pose_telemetry(self, isar_id, robot_name: str) -> str:
        pose_payload = TelemetryPosePayload(
            isar_id=isar_id,
            robot_name=robot_name,
            timestamp=datetime.now(UTC),
            pose=self._get_robot_pose_in_asset_frame(),
        )
        return pose_payload.model_dump_json()

    def _get_battery_telemetry(self, isar_id, robot_name: str) -> str:
        battery_value_list = self.telemetry.get_robot_battery()
        battery_payload = TelemetryBatteryPayload(
            isar_id=isar_id,
            robot_name=robot_name,
            timestamp=datetime.now(UTC),
            battery_level=battery_value_list[0],
            battery_state=battery_value_list[1],
        )
        return battery_payload.model_dump_json()

    def _get_obstacle_status_telemetry(self, isar_id: str, robot_name: str) -> str:
        obstacle_status_payload = TelemetryObstacleStatusPayload(
            isar_id=isar_id,
            robot_name=robot_name,
            timestamp=datetime.now(UTC),
            obstacle_status=self.telemetry.get_robot_obstacle_status(),
        )
        return obstacle_status_payload.model_dump_json()

    def _get_pressure_telemetry(self, isar_id, robot_name: str) -> str:
        pressure_payload = TelemetryPressurePayload(
            isar_id=isar_id,
            robot_name=robot_name,
            timestamp=datetime.now(UTC),
            pressure_level=self.telemetry.get_robot_pressure_level(),
        )
        return pressure_payload.model_dump_json()

    def _get_robot_pose_in_asset_frame(self) -> Pose:
        taurob_pose: TaurobPose = self.telemetry.get_robot_pose()
        return self.map_handler.transform_pose_to_asset(taurob_pose)

    def robot_status(self) -> RobotStatus:
        now: datetime = datetime.now().astimezone(tz=pytz.utc)
        time_since_last_update: float = (
            now - self.robot_api.last_update_from_robot
        ).total_seconds()
        # Converts from milliseconds to seconds, as the received value is 1000 too large

        if time_since_last_update > settings.ROBOT_REQUESTS_TIMEOUT:
            return RobotStatus.Offline

        if self.telemetry.get_robot_remote_stop():
            return RobotStatus.TeleOperation

        if self._get_is_docked() or self.confirmed_docked_since_last_mission:
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

        # We can reuse the localization action if it never completed
        need_new_initial_action = (
            not len(
                list(
                    filter(
                        lambda action: action["status"] == "canceled"
                        and action["mission_step"] is not None
                        and action["mission_step"]["step_type"] == "start",
                        body,
                    )
                )
            )
            > 0
        )

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

        if need_new_initial_action:
            localization_action_id = self._create_initial_action()
            action_ids_list.insert(0, localization_action_id)
            self.logger.info("Adding new initial action.")

        index_of_cancelled_step = len(body) - 1 - len(action_ids_list)
        # -1 accounts for the fact that we have mission options introduced as an action

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
        return None

    def get_battery_level(self) -> float:
        return self.telemetry.last_battery_measurement
