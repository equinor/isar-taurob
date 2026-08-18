import datetime
import json
import logging
import uuid
from dataclasses import asdict
from enum import Enum
from http import HTTPStatus
from typing import Any

import pytz
from requests import HTTPError
from requests.exceptions import JSONDecodeError, RequestException, Timeout
from requests.models import Response
from robot_interface.models.exceptions.robot_exceptions import (
    RobotAPIException,
    RobotCommunicationException,
    RobotCommunicationTimeoutException,
)

from isar_taurob.config import settings
from isar_taurob.robot.api.measurement_metadata import MeasurementMetadata
from isar_taurob.robot.api.request_handler import RequestHandler
from isar_taurob.robot.taurob_model.arm import Joints
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose


class ActionType(str, Enum):
    localize = "start"
    measure = "measure"
    driveTo = "drive"


class API:
    TAUROB_TIME_FORMAT: str = "%Y-%m-%dT%H:%M:%S.%f%z %Z"

    def __init__(
        self,
        request_handler: RequestHandler,
        host: str = settings.ROBOT_API_HOST,
        auth: tuple = (settings.API_USERNAME, settings.API_PASSWORD),
        robot_name: str = settings.ROBOT_NAME,
        mission_definition_endpoint: str = settings.ROBOT_MISSION_DEFINITION_ENDPOINT,
        mission_schedule_endpoint: str = settings.ROBOT_MISSION_SCHEDULE_ENDPOINT,
        robot_action_endpoint: str = settings.ROBOT_ACTION_ENDPOINT,
        request_timeout: float = settings.ROBOT_REQUESTS_TIMEOUT,
        robot_endpoint: str = settings.ROBOT_ENDPOINT,
        robot_mission_editor_endpoint: str = settings.ROBOT_MISSION_EDITOR_ENDPOINT,
    ):
        self.request_handler = request_handler
        self.host = host
        self.auth = auth
        self.robot_name = robot_name
        self.request_timeout = request_timeout
        self.logger = logging.getLogger(settings.LOGGER_NAME)
        self.last_update_from_robot: datetime.datetime = datetime.datetime(
            year=datetime.MINYEAR, month=1, day=1, tzinfo=pytz.utc
        )

        self.base_url_robot = self.host + robot_endpoint
        self.base_url_mission_definition = self.host + mission_definition_endpoint
        self.base_url_mission_schedule = self.host + mission_schedule_endpoint
        self.base_url_robot_action = self.host + robot_action_endpoint
        self.base_url_editor = self.host + robot_mission_editor_endpoint

    def get_sensor_value(self, sensor_name: str) -> dict:
        """
        :param sensor_name: Name of sensor (e.g. "light")
        :return: Returns a dict containing sensor value and metadata
        """
        url: str = f"{self.base_url_robot}/{self.robot_name}/sensor/{sensor_name}"
        try:
            response = self.request_handler.get(url)
        except RequestException as e:
            error_description: str = (
                f"An error of type {e.__class__.__name__} occurred while retrieving sensor value '{sensor_name}' from robot API"
            )
            raise RobotCommunicationException(error_description=error_description)

        body: dict = self._unpack_json(response, dict)

        return body

    def update_last_robot_update_time(self, sensor_response_body: dict):
        if sensor_response_body["changed"] is None:
            return

        try:
            last_update_of_sensor: datetime.datetime = datetime.datetime.strptime(
                sensor_response_body["changed"], API.TAUROB_TIME_FORMAT
            ).replace(tzinfo=datetime.UTC)
        except ValueError as e:
            self.logger.warning(
                f"Could not parse time string '{sensor_response_body['changed']}' from robot API with error message: {e}. Will return preivious value of last update time."
            )
            return self.last_update_from_robot

        self.last_update_from_robot = max(
            self.last_update_from_robot, last_update_of_sensor
        )
        return last_update_of_sensor

    def schedule(self, mission_id: int, step_list) -> int:
        url = (
            f"{self.base_url_mission_definition}/{mission_id}"
            f"/schedule/{self.robot_name}"
        )
        try:
            # TODO Find suitable place to set request timeout for schedule call
            response = self.request_handler.post(
                url=url, json_body=step_list, request_timeout=300
            )
        except HTTPError as e:
            error_description: str = (
                f"An HTTP Error occurred with status "
                f"code {e.response.status_code if e.response else None}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

        except RequestException as e:
            error_description = (
                f"An error occurred during the mission scheduling to the robot: {e}"
            )
            raise RobotCommunicationException(error_description=error_description)

        body: dict = self._unpack_json(response, dict)
        try:
            return body["instance_id"]
        except KeyError:
            error_description = (
                "An error occurred while accessing the"
                " schedule mission return dictionary"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def get_mission_instance_details(self, instance_id: int) -> list:
        try:
            response = self.request_handler.get(
                f"{self.base_url_mission_schedule}/{str(instance_id)}"
            )
        except Timeout:
            error_description: str = (
                "A timeout occurred while requesting the mission details from the robot"
            )
            self.logger.exception(error_description)
            raise RobotCommunicationTimeoutException(
                error_description=error_description
            )
        except RequestException:
            error_description = f'An error occurred while collecting the mission details for instance_id "{instance_id}" from the robot'
            self.logger.exception(error_description)
            raise RobotCommunicationException(error_description=error_description)

        return self._unpack_json(response, list)

    def get_url_data(self, url: str) -> Response | None:
        try:
            response = self.request_handler.get(url)
            return response
        except RequestException:
            error_description: str = (
                f"An error occurred while retrieving the data for url: {url}"
            )
            self.logger.exception(error_description)
            raise RobotCommunicationException(error_description=error_description)

    def get_scheduled_missions(self) -> list:
        try:
            response = self.request_handler.get(self.base_url_mission_schedule)
        except RequestException:
            error_description: str = (
                "An error occurred while collecting scheduled missions"
            )
            self.logger.exception(error_description)
            raise RobotCommunicationException(error_description=error_description)

        body: list = self._unpack_json(response, list)

        instance_ids = []
        for instance in body:
            try:
                if instance["robot"] == self.robot_name:
                    instance_ids += [instance["instance_id"]]
            except KeyError:
                self.logger.error("Response does not contain key 'instance_id'")
        return instance_ids

    def get_scheduled_actions(self, robot_name: str) -> list:
        # The reason this function uses the requests package and not the request_handler
        # is that the output from the API gives a 404 Not Found when an empty list is
        # returned. This is not ideal, but a workaround such that we may avoid logging
        # exceptions constantly when the functionality is behaving as expected.
        try:
            # with warnings.catch_warnings():
            #     warnings.simplefilter("ignore")
            #     response: Response = requests.get(
            #         url=f"{self.base_url_robot_action}/list/{robot_name}",
            #         auth=self.auth,
            #         verify=False,
            #     )
            response = self.request_handler.get(
                f"{self.base_url_robot_action}/list/{robot_name}"
            )

        except RequestException as e:
            if e.response.status_code == HTTPStatus.NOT_FOUND:
                return []
            error_description: str = (
                f"A request error occurred while collecting scheduled actions: {e}"
            )
            self.logger.error(error_description)
            raise RobotCommunicationException(error_description=error_description)

        try:
            response.raise_for_status()
        except HTTPError as e:
            if e.response.status_code == HTTPStatus.NOT_FOUND:
                return []

            error_description = (
                f"A HTTP Error occurred with status code {e.response.status_code}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

        if response.status_code == HTTPStatus.NO_CONTENT:
            return []

        return response.json()

    def delete_all_mission_instances(self) -> None:
        instance_ids = self.get_scheduled_missions()

        if len(instance_ids) == 0:
            self.logger.warning(
                "No mission were found when trying to cancel all scheduled missions"
            )

        for instance_id in instance_ids:
            self.delete_mission_instance(instance_id)

    def delete_mission_instance(self, instance_id: int) -> None:
        self.logger.info(
            f"Sending API request to stop mission instance with id {instance_id}"
        )
        try:
            self.request_handler.delete(
                f"{self.base_url_mission_schedule}/{instance_id}"
            )
        except RequestException:
            error_description: str = (
                "A request error occurred while deleting mission instances"
            )
            self.logger.error(error_description)
            raise RobotCommunicationException(error_description=error_description)

    def set_measurement_metadata(
        self, step_id: int, measurement_metadata: MeasurementMetadata
    ) -> None:
        url: str = f"{self.host}/mission/step/{step_id}/metadata"
        body: dict = {
            "measurement": asdict(
                measurement_metadata, dict_factory=MeasurementMetadata.dict_factory
            )
        }
        try:
            self.request_handler.post(url, json_body=body)
        except RequestException:
            error_description: str = (
                f"A request error occurred while setting measurement "
                f"metadata for step {step_id}"
            )
            self.logger.error(error_description)
            raise RobotCommunicationException(error_description=error_description)

    def amend_mission(
        self,
        mission_name: str,
        action_ids: list[int],
    ):

        url: str = f"{self.base_url_editor}/mission/definition/{settings.MISSION_ID}"
        body: dict = {
            "MissionSteps": action_ids,
            "MissionName": mission_name + " " + str(uuid.uuid4()),
        }

        try:
            response: Response = self.request_handler.post(url, json_body=body)
        except RequestException:
            error_description: str = "A request error occurred while amending mission"
            self.logger.error(error_description)
            raise RobotCommunicationException(error_description=error_description)

        try:
            response.raise_for_status()
        except HTTPError as e:
            if e.response.status_code == HTTPStatus.NOT_FOUND:
                return []

            error_description = (
                f"An HTTP Error occurred with status code {e.response.status_code}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

    def _get_robot_pose_json(
        self,
        robot_name: str,
        map_name: str,
        pose: TaurobPose,
        joints: Joints | None = None,
    ):
        arm_pose_json: str = ""
        base_pose: str = f""" \
            {{ \
                "x":{pose.x}, \
                "y":{pose.y}, \
                "theta":{pose.theta}, \
                "floor":"{map_name}", \
                "flipper_joint":{settings.FLIPPER_JOINT} \
            }}"""

        if joints is not None:
            if settings.ROBOT_NUMBER_OF_JOINTS == 2:
                joints.j3 = -1
                joints.j4 = -1

            arms_pose: str = f""" \
                 {{ \
                    "joint_positions": \
                        {{ \
                            "arm_joint_1":{joints.j1}, \
                            "arm_joint_2":{joints.j2}, \
                            "arm_joint_3":{joints.j3}, \
                            "arm_joint_4":{joints.j4} \
                        }} \
                }} """

            arm_pose_json = f""""arm_pose": {json.dumps(arms_pose)},"""
        return f""" \
            {{ \
                "{robot_name}": \
                    {{ \
                        {arm_pose_json} \
                        "base_pose": {json.dumps(base_pose)} \
                    }} \
            }} \
        """

    def create_action(
        self,
        name: str,
        type: ActionType,
        pose: TaurobPose,
        map_name: str,
        joints: Joints | None = None,
        metadata: MeasurementMetadata | None = None,
    ):
        url: str = f"{self.base_url_editor}/editor/action"
        metadata_dict = (
            f"""{{"measurement": {json.dumps(asdict(metadata, dict_factory=MeasurementMetadata.dict_factory))}}}"""
            if metadata is not None
            else ""
        )

        robot_pose: str = self._get_robot_pose_json(
            self.robot_name, map_name, pose, joints=joints
        )

        body: dict = {
            "ActionName": name,
            "ActionType": type.value,
            "Metadata": metadata_dict,
            "RobotPose": robot_pose,
            "ActionInfo": "",
        }

        try:
            response: Response = self.request_handler.post(url, json_body=body)
        except RequestException as e:
            error_description: str = (
                f"A request error occurred while creating an action with response: '{e.response}'"
            )
            self.logger.error(error_description)
            raise RobotCommunicationException(error_description=error_description)

        try:
            response.raise_for_status()
        except HTTPError as e:
            error_description = (
                f"An HTTP Error occurred with status code {e.response.status_code}"
            )
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

        json_response = self._unpack_json(response, dict)

        return json_response["ActionID"]

    def _unpack_json(self, response: Response, expected_unpacked_type: Any) -> Any:
        try:
            body = response.json()
        except JSONDecodeError:
            error_description: str = "An error occurred while unpacking the response"
            self.logger.exception(error_description)
            raise RobotAPIException(error_description=error_description)

        if isinstance(body, expected_unpacked_type):
            return body
        else:
            error_description = (
                f"Response is not of expected type. "
                f"Got: {type(body)} Expected: {expected_unpacked_type}"
            )
            self.logger.error(error_description)
            raise RobotAPIException(error_description=error_description)
