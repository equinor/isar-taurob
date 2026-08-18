from pathlib import Path
from typing import Literal


class MockTaurobAPI:
    @staticmethod
    def get_scheduled_missions(
        return_name: Literal["success", "success_empty"] = "success",
    ):
        if return_name == "success":
            return [
                {"instance_id": 232, "name": "Mini Mission", "robot": "ltdata"},
                {"instance_id": 195, "name": "Mini Mission", "robot": "ltdata"},
            ]
        elif return_name == "success_empty":
            return []

    @staticmethod
    def schedule(return_name: Literal["success"] = "success"):
        if return_name == "success":
            return {"instance_id": 232}

    @staticmethod
    def get_mission_details(instance_id: Literal[232, 195, -1, 123, 124, 125] = 232):
        if instance_id == 232:
            return [
                {
                    "instance_id": 232,
                    "status": "scheduled",
                    "measurement_results": [],
                    "mission_step": {
                        "mission_id": 10275,
                        "step_number": 1,
                        "step_id": 300,
                        "step_description": "Start Mission",
                        "step_type": "start",
                        "tag": None,
                        "sensor_name": None,
                        "sensor_result": None,
                        "pose": '{"ltdata": {"base_pose": "{\\"floor\\": \\"level1\\",'
                        + ' \\"theta\\": -0.182, \\"x\\": -5.125, \\"y\\": -5.17}", "arm_pose": "{}"}}',
                    },
                }
            ]
        elif instance_id == 195:
            return [
                {
                    "instance_id": 195,
                    "status": "scheduled",
                    "measurement_results": [],
                    "mission_step": {
                        "mission_id": 10275,
                        "step_number": 3,
                        "step_id": 302,
                        "step_description": "Drive Back",
                        "step_type": "drive",
                        "tag": None,
                        "sensor_name": None,
                        "sensor_result": None,
                        "pose": '{"ltdata": {"base_pose": "{\\"x\\": -5.0, \\"y\\": -5.0, '
                        + '\\"theta\\": 0.0, \\"floor\\": \\"floor_1\\"}"}}',
                    },
                },
            ]
        elif instance_id == -1:
            return []
        elif instance_id == 123:
            return [
                {
                    "instance_id": 123,
                    "status": "successful",
                    "measurement_results": [],
                    "mission_step": None,
                },
                {
                    "instance_id": 1235,
                    "status": "successful",
                    "measurement_results": [
                        {
                            "url": "https://127.0.0.1:8443/basic/fleet/measurement/10248",
                            "media_type": "image/jpeg",
                        }
                    ],
                    "mission_step": {
                        "step_number": 0,
                        "step_id": 1234,
                        "step_type": "measure",
                        "metadata": {
                            "measurement": {
                                "sensor_name": "camera_sensorhead",
                                "result_type": "still",
                                "duration": 0,
                                "result_format": "image/jpeg",
                            }
                        },
                    },
                },
            ]
        elif instance_id == 124:
            return [
                {
                    "instance_id": 124,
                    "status": "unsuccessful",
                    "measurement_results": [],
                    "mission_step": None,
                },
                {
                    "instance_id": 1235,
                    "status": "unsuccessful",
                    "measurement_results": [
                        {
                            "url": "https://127.0.0.1:8443/basic/fleet/measurement/10249",
                            "media_type": "image/jpeg",
                        }
                    ],
                    "mission_step": {
                        "step_number": 0,
                        "step_id": 1234,
                        "step_type": "measure",
                        "metadata": {
                            "measurement": {
                                "sensor_name": "camera_sensorhead",
                                "result_type": "still",
                                "duration": 0,
                                "result_format": "image/jpeg",
                            }
                        },
                    },
                },
            ]
        elif instance_id == 125:
            return [
                {
                    "instance_id": 125,
                    "status": "successful",
                    "measurement_results": [
                        {
                            "url": "https://127.0.0.1:8443/basic/fleet/measurement/10250",
                            "media_type": "video/mp4",
                        }
                    ],
                    "mission_step": {
                        "step_number": 0,
                        "step_id": 1234,
                        "step_type": "measure",
                        "metadata": {
                            "measurement": {
                                "sensor_name": "camera_thermal_g300a",
                                "result_type": "video",
                                "duration": 10,
                                "result_format": "video/mp4",
                            }
                        },
                    },
                },
            ]

    @staticmethod
    def get_sensor_value(
        return_name: Literal[
            "light",
            "robot_remote_stop",
            "sensor.obstacle_status",
            "pose.base",
            "pose.arm",
            "pose.total_distance",
            "health.battery_level",
            "health.stop_reasons",
            "sensor.gpa",
            "sensor.docking_state",
            "sensor.pcu.pressure_1",
            "sensor.pcu.pressure_2",
            "sensor.pcu.wireless_emergency_stop",
            "sensor.pcu.atex_mode",
            "sensor.pcu.battery_warning",
            "user_action_request",
            "health.uecu.base_l_flipper.supply_voltage",
            "health.uecu.base_l_flipper.temperature",
            "health.uecu.base_r_j1.supply_voltage",
            "health.uecu.base_r_j1.temperature",
            "health.uecu.j2_j3.supply_voltage",
            "health.uecu.j2_j3.temperature",
            "health.uecu.j4_j5.supply_voltage",
            "health.uecu.j4_j5.temperature",
            "last_mission_failure_reason",
        ] = "pose.base",
    ) -> dict:
        if return_name == "light":
            return {"name": "light", "value": None, "changed": None, "version": 0}
        elif return_name == "robot_remote_stop":
            return {
                "name": "robot_remote_stop",
                "value": False,
                "changed": "2021-01-18T11:36:20.786061+00:00 UTC",
                "version": 42885,
            }
        elif return_name == "pose.base":
            return {
                "name": "pose.base",
                "value": '{"x": 0.01, "y": 0.025, "theta": 6.283, "flipper_joint": 6.28, "floor": "test_map"}',
                "changed": "2020-12-08T14:36:08.496650+00:00 UTC",
                "version": 14610,
            }
        elif return_name == "pose.arm":
            return {
                "name": "pose.arm",
                "value": '{"arm_joint_1": 4.021, "arm_joint_2": 0.108}',
                "changed": "2021-01-18T11:36:20.706515+00:00 UTC",
                "version": 81840,
            }
        elif return_name == "pose.total_distance":
            return {
                "name": "pose.total_distance",
                "value": 0,
                "changed": "2021-01-18T11:36:20.562458+00:00 UTC",
                "version": 42700,
            }
        elif return_name == "health.battery_level":
            return {
                "name": "health.battery_level",
                "value": 10,
                "changed": "2021-01-18T11:36:20.786061+00:00 UTC",
                "version": 42885,
            }
        elif return_name == "sensor.obstacle_status":
            return {
                "name": "sensor.obstacle_status",
                "value": "{'x': 20, 'y':20 }",
                "changed": "2023-08-18T11:36:20.786061+00:00 UTC",
                "version": 42885,
            }
        elif return_name == "sensor.pcu.wireless_emergency_stop":
            return {
                "name": "sensor.pcu.wireless_emergency_stop",
                "value": False,
                "changed": "2021-01-18T11:36:20.874528+00:00 UTC",
                "version": 42877,
            }
        elif return_name == "sensor.pcu.atex_mode":
            return {
                "name": "sensor.pcu.atex_mode",
                "value": False,
                "changed": "2021-01-18T11:36:20.921740+00:00 UTC",
                "version": 42839,
            }
        elif return_name == "sensor.pcu.battery_warning":
            return {
                "name": "sensor.pcu.battery_warning",
                "value": '{"warning": false}',
                "changed": "2021-01-18T11:36:20.831076+00:00 UTC",
                "version": 42887,
            }
        elif return_name == "user_action_request":
            return {
                "name": "user_action_request",
                "value": None,
                "changed": None,
                "version": 0,
            }
        elif return_name == "health.uecu.base_l_flipper.supply_voltage":
            return {
                "name": "health.uecu.base_l_flipper.supply_voltage",
                "value": 24.601,
                "changed": "2021-01-18T11:36:20.696583+00:00 UTC",
                "version": 42734,
            }
        elif return_name == "health.uecu.base_l_flipper.temperature":
            return {
                "name": "health.uecu.base_l_flipper.temperature",
                "value": 32,
                "changed": "2021-01-18T11:36:20.741655+00:00 UTC",
                "version": 42720,
            }
        elif return_name == "health.uecu.base_r_j1.supply_voltage":
            return {
                "name": "health.uecu.base_r_j1.supply_voltage",
                "value": 24.459,
                "changed": "2021-01-18T11:36:20.607203+00:00 UTC",
                "version": 42740,
            }
        elif return_name == "health.uecu.base_r_j1.temperature":
            return {
                "name": "health.uecu.base_r_j1.temperature",
                "value": 32,
                "changed": "2021-01-18T11:36:20.652291+00:00 UTC",
                "version": 42741,
            }
        elif return_name == "health.uecu.j2_j3.supply_voltage":
            return {
                "name": "health.uecu.j2_j3.supply_voltage",
                "value": None,
                "changed": None,
                "version": 0,
            }
        elif return_name == "health.uecu.j2_j3.temperature":
            return {
                "name": "health.uecu.j2_j3.temperature",
                "value": None,
                "changed": None,
                "version": 0,
            }
        elif return_name == "health.uecu.j4_j5.supply_voltage":
            return {
                "name": "health.uecu.j4_j5.supply_voltage",
                "value": None,
                "changed": None,
                "version": 0,
            }
        elif return_name == "health.uecu.j4_j5.temperature":
            return {
                "name": "health.uecu.j4_j5.temperature",
                "value": None,
                "changed": None,
                "version": 0,
            }
        elif return_name == "last_mission_failure_reason":
            return {
                "name": "last_mission_failure_reason",
                "value": '{"failure_reason": "Error executing \'wallbox_dock\' action: Too far away from  wallbox to start docking. (1.53 m to reference, limit is 1.50 m.)", "action_id": 62405}',
                "changed": None,
                "version": 0,
            }
        else:
            return {
                "type": "about:blank",
                "title": "No such sensor.",
                "detail": "Nothing matches the given URI",
                "status": 404,
            }

    @staticmethod
    def get_url_data():
        class MockResponse:
            def __init__(self, headers, content):
                self.headers = headers
                self.content = content

        image_path = (
            Path(__file__)
            .resolve()
            .parent.joinpath(
                Path("../../test_data/mock_mission_report/measurement-0.jpeg")
            )
        )

        with open(image_path, "rb") as image:
            response = MockResponse(
                {
                    "Content-Type": "image/jpeg",
                    "Robot-Pose": {
                        "base_pose": {
                            "position": {"x": 17.537, "y": 50.123, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.016},
                            "x": 17.537,
                            "y": 50.123,
                            "theta": 3.109,
                            "flipper_joint": -0.053,
                            "floor": "test_map",
                        }
                    },
                },
                bytearray(image.read()),
            )

            return response
