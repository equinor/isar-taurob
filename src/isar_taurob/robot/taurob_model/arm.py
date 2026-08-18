import logging
import math

import numpy as np
from alitra import Position
from robot_interface.models.exceptions.robot_exceptions import (
    RobotException,
    RobotInfeasibleTaskException,
)
from scipy.spatial.transform import Rotation

from isar_taurob.config import settings
from isar_taurob.robot.taurob_model.joints import Joints
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose


class TaurobArm:
    def __init__(self, number_of_joints) -> None:
        self.logger = logging.getLogger(settings.LOGGER_NAME)

        self.number_of_joints: int = number_of_joints
        self.link_1: float
        self.link_2: float
        if number_of_joints == 2:
            self.link_1 = settings.TWO_DOF_ARM_LINK_1
            self.link_2 = settings.TWO_DOF_ARM_LINK_2
            self.arm_placement = np.array(
                [
                    settings.TWO_DOF_ARM_BASE_X,
                    settings.TWO_DOF_ARM_BASE_Y,
                    settings.TWO_DOF_ARM_BASE_Z,
                ]
            )
        elif number_of_joints == 4:
            self.link_1 = settings.FOUR_DOF_ARM_LINK_1
            self.link_2 = settings.FOUR_DOF_ARM_LINK_2
            self.link_3: float = settings.FOUR_DOF_ARM_LINK_3
            self.link_4: float = settings.FOUR_DOF_ARM_LINK_4
            self.arm_placement = np.array(
                [
                    settings.FOUR_DOF_ARM_BASE_X,
                    settings.FOUR_DOF_ARM_BASE_Y,
                    settings.FOUR_DOF_ARM_BASE_Z,
                ]
            )
        else:
            error_description: str = (
                f"An invalid number of joints "
                f"({settings.ROBOT_NUMBER_OF_JOINTS}) was configured"
            )
            self.logger.error(error_description)
            raise RobotException(error_description=error_description)

        self.joint_1_at_zero: float = settings.TWO_DOF_JOINT_1_AT_ZERO_YAW
        self.joint_2_at_vertical_arm: float = settings.TWO_DOF_JOINT_2_AT_VERTICAL_ARM
        self.number_of_joints = number_of_joints

    @staticmethod
    def _rotation_arm_to_navigation_frame(theta: float) -> np.ndarray:
        """Get rotation matrix for arm frame to navigation frame"""
        return Rotation.from_euler("z", theta).as_matrix()

    @staticmethod
    def _rotation_navigation_to_arm_frame(theta: float) -> np.ndarray:
        """Get rotation matrix for navigation frame to arm frame"""
        return np.transpose(Rotation.from_euler("z", theta).as_matrix())

    def _check_feasible_inspection(
        self, distance: float, minimum_distance: float
    ) -> None:
        """Check feasible inspection"""
        if distance < minimum_distance:
            error_description: str = (
                "The target is too close to the robot base position"
            )
            self.logger.error(error_description)
            raise RobotInfeasibleTaskException(error_description=error_description)

    def _calc_arm_position_in_navigation_frame(
        self, robot_pose: TaurobPose
    ) -> Position:
        """Calculate arm position in navigation frame frame, given robot pose"""
        robot_in_navigation_frame = robot_pose.to_pose().position.to_array()
        arm_in_navigation_frame: np.ndarray = (
            robot_in_navigation_frame
            + TaurobArm._rotation_arm_to_navigation_frame(theta=robot_pose.theta)
            @ self.arm_placement
        )
        return Position.from_array(position=arm_in_navigation_frame, frame="navigation")

    def _calc_point_in_arm_frame(
        self, robot_pose: TaurobPose, point_in_navigation_frame: Position
    ) -> Position:
        """Calculate point in arm frame, given point in navigation frame and robot pose"""
        arm_in_navigation_frame: Position = self._calc_arm_position_in_navigation_frame(
            robot_pose=robot_pose
        )
        point_in_arm_frame: np.ndarray = TaurobArm._rotation_navigation_to_arm_frame(
            theta=robot_pose.theta
        ) @ (point_in_navigation_frame.to_array() - arm_in_navigation_frame.to_array())

        return Position.from_array(position=point_in_arm_frame, frame="arm")

    def _calculate_joint_4_pos_in_arm_frame(
        self, joint_2: float, joint_3: float
    ) -> Position:
        joint_4_pos_in_arm_frame_x: float = self.link_2 * math.cos(
            joint_2 - settings.FOUR_DOF_JOINT_2_AT_ZERO_POSITION
        ) + self.link_3 * math.cos(
            joint_2
            - settings.FOUR_DOF_JOINT_2_AT_ZERO_POSITION
            + joint_3
            - settings.FOUR_DOF_JOINT_3_AT_ZERO_POSITION
            - math.pi
        )

        joint_4_pos_in_arm_frame_y: float = 0
        joint_4_pos_in_arm_frame_z: float = self.link_2 * math.sin(
            joint_2 - settings.FOUR_DOF_JOINT_2_AT_ZERO_POSITION
        ) + self.link_3 * math.sin(
            joint_2
            - settings.FOUR_DOF_JOINT_2_AT_ZERO_POSITION
            + joint_3
            - settings.FOUR_DOF_JOINT_3_AT_ZERO_POSITION
            - math.pi
        )

        joint_4_pos_in_arm_frame: np.ndarray = np.array(
            [
                joint_4_pos_in_arm_frame_x,
                joint_4_pos_in_arm_frame_y,
                joint_4_pos_in_arm_frame_z,
            ]
        )

        return Position.from_array(position=joint_4_pos_in_arm_frame, frame="arm")

    def _calculate_vector_from_joint_4_to_target_in_arm_frame(
        self, target_in_arm_frame: Position, joint_4_pos_in_arm_frame: Position
    ) -> np.ndarray:
        return target_in_arm_frame.to_array() - joint_4_pos_in_arm_frame.to_array()

    def _calculate_target_in_arm_frame_after_rotating_joint_1(
        self, joint_1: float, target_in_arm_frame: Position
    ) -> Position:
        target_in_arm_frame_after_rotating_joint_1: Position = (
            Rotation.from_euler(
                "z", settings.FOUR_DOF_JOINT_1_AT_ZERO_POSITION - joint_1
            ).as_matrix()
            @ target_in_arm_frame.to_array()
        )
        return Position.from_array(
            position=target_in_arm_frame_after_rotating_joint_1, frame="arm"
        )

    def _calculate_angle_between_link_4_and_arm_frame_x_axis(
        self, vector_from_joint_4_to_target_in_arm_frame: np.ndarray
    ) -> float:
        x_distance_from_joint_4_to_target_in_arm_frame: float = (
            vector_from_joint_4_to_target_in_arm_frame[0]
        )

        z_distance_from_joint_4_to_target_in_arm_frame: float = (
            vector_from_joint_4_to_target_in_arm_frame[2]
        )

        distance_from_joint_4_to_target_in_arm_frame: float = math.sqrt(
            x_distance_from_joint_4_to_target_in_arm_frame**2
            + z_distance_from_joint_4_to_target_in_arm_frame**2
        )

        angle_between_joint_4_pos_and_target_pos: float = math.atan2(
            z_distance_from_joint_4_to_target_in_arm_frame,
            x_distance_from_joint_4_to_target_in_arm_frame,
        )

        angle_between_joint_4_to_target_and_link_4: float = math.acos(
            self.link_4 / distance_from_joint_4_to_target_in_arm_frame
        )
        angle_between_link_4_and_arm_frame_x_axis: float = (
            angle_between_joint_4_pos_and_target_pos
            + angle_between_joint_4_to_target_and_link_4
        )

        return angle_between_link_4_and_arm_frame_x_axis

    def _solve_inverse_kinematics_joint_1(self, target_in_arm_frame: Position) -> float:
        """Solve inverse kinematics joint 1

        x/y/z_at: x/y/z-component of target in arm frame
        """
        x_at, y_at, _ = target_in_arm_frame.to_array()
        xy_distance_arm_to_target: float = math.sqrt(x_at**2 + y_at**2)

        self._check_feasible_inspection(
            distance=xy_distance_arm_to_target, minimum_distance=self.link_1
        )

        heading_angle_arm_frame_to_target = math.atan2(y_at, x_at)
        offset_angle_from_link_length = math.asin(
            self.link_1 / xy_distance_arm_to_target
        )

        joint_1: float = (
            self.joint_1_at_zero
            + heading_angle_arm_frame_to_target
            + offset_angle_from_link_length
        )
        return joint_1

    def _solve_inverse_kinematics_joint_2(self, target_in_arm_frame: Position) -> float:
        """Solve inverse kinematics joint 2

        x/y/z_at: x/y/z-component of target in arm frame
        """
        x_at, y_at, z_at = target_in_arm_frame.to_array()
        xy_distance_arm_to_target: float = math.sqrt(x_at**2 + y_at**2)
        xy_distance_link_1_tip_to_target: float = math.sqrt(
            xy_distance_arm_to_target**2 - self.link_1**2
        )
        distance_link_1_tip_to_target: float = math.sqrt(
            xy_distance_link_1_tip_to_target**2 + z_at**2
        )

        self._check_feasible_inspection(
            distance=distance_link_1_tip_to_target, minimum_distance=self.link_2
        )

        pitch_angle_link_1_tip_to_target = math.atan2(
            z_at, xy_distance_link_1_tip_to_target
        )
        offset_angle_from_link_length = math.asin(
            self.link_2 / distance_link_1_tip_to_target
        )

        joint_2: float = (
            self.joint_2_at_vertical_arm
            + pitch_angle_link_1_tip_to_target
            - offset_angle_from_link_length
        )

        return joint_2

    def solve_inverse_kinematics(
        self, robot_pose: TaurobPose, target: Position
    ) -> Joints:
        """Both robot_pose and target are in navigation frame"""
        if self.number_of_joints == 2:
            return self._solve_inverse_kinematics_two_dof(robot_pose, target)
        elif self.number_of_joints == 4:
            return self._solve_inverse_kinematics_four_dof(robot_pose, target)
        else:
            error_description: str = (
                f"An invalid number of joints "
                f"({settings.ROBOT_NUMBER_OF_JOINTS}) was configured"
            )
            self.logger.error(error_description)
            raise RobotInfeasibleTaskException(error_description=error_description)

    def _solve_inverse_kinematics_two_dof(
        self,
        robot_pose: TaurobPose,
        target_in_navigation_frame: Position,
    ) -> Joints:
        """Solve inverse kinematics for two degrees of freedom arm"""
        target_in_arm_frame = self._calc_point_in_arm_frame(
            robot_pose=robot_pose, point_in_navigation_frame=target_in_navigation_frame
        )

        joint_1 = self._solve_inverse_kinematics_joint_1(
            target_in_arm_frame=target_in_arm_frame
        )

        joint_2 = self._solve_inverse_kinematics_joint_2(
            target_in_arm_frame=target_in_arm_frame
        )

        return Joints(j1=joint_1, j2=joint_2, number_of_joints=self.number_of_joints)

    def _solve_inverse_kinematics_joint_1_four_dof(
        self, target_in_arm_frame: Position
    ) -> float:
        """Solve inverse kinematics for joint 1 for four degrees of freedom arm

        x/y/z_at: x/y/z-component of target in arm frame
        """
        x_at, y_at, _ = target_in_arm_frame.to_array()
        heading_angle_arm_frame_to_target: float = math.atan2(y_at, x_at)
        joint_1: float = (
            settings.FOUR_DOF_JOINT_1_AT_ZERO_POSITION
            + heading_angle_arm_frame_to_target
        )

        normalized_joint_1 = self._normalize(joint_1)

        return normalized_joint_1

    def _solve_inverse_kinematics_joint_4_four_dof(
        self, joint_2: float, joint_3: float, target_in_arm_frame: Position
    ) -> float:
        """Solve inverse kinematics for joint 4 for four degrees of freedom arm"""

        joint_4_pos_in_arm_frame: Position = self._calculate_joint_4_pos_in_arm_frame(
            joint_2=joint_2, joint_3=joint_3
        )
        vector_from_joint_4_to_target_in_arm_frame: np.ndarray = (
            self._calculate_vector_from_joint_4_to_target_in_arm_frame(
                target_in_arm_frame=target_in_arm_frame,
                joint_4_pos_in_arm_frame=joint_4_pos_in_arm_frame,
            )
        )

        angle_between_link_4_and_arm_frame_x_axis: float = (
            self._calculate_angle_between_link_4_and_arm_frame_x_axis(
                vector_from_joint_4_to_target_in_arm_frame=vector_from_joint_4_to_target_in_arm_frame
            )
        )

        joint_4: float = (
            joint_2
            - settings.FOUR_DOF_JOINT_2_AT_ZERO_POSITION
            + joint_3
            - settings.FOUR_DOF_JOINT_3_AT_ZERO_POSITION
            - angle_between_link_4_and_arm_frame_x_axis
            + settings.FOUR_DOF_JOINT_4_AT_ZERO_POSITION
        )

        normalized_joint_4 = self._normalize(joint_4)

        return normalized_joint_4

    def _solve_inverse_kinematics_four_dof(
        self, robot_pose: TaurobPose, target_in_navigation_frame: Position
    ) -> Joints:
        """Solve inverse kinematics for four degrees of freedom arm"""

        target_in_arm_frame: Position = self._calc_point_in_arm_frame(
            robot_pose=robot_pose, point_in_navigation_frame=target_in_navigation_frame
        )
        joint_1: float = self._solve_inverse_kinematics_joint_1_four_dof(
            target_in_arm_frame=target_in_arm_frame
        )

        target_in_arm_frame_after_rotating_joint_1: Position = (
            self._calculate_target_in_arm_frame_after_rotating_joint_1(
                joint_1=joint_1, target_in_arm_frame=target_in_arm_frame
            )
        )

        joint_2: float = 2.637
        joint_3: float = 1.701

        joint_4: float = self._solve_inverse_kinematics_joint_4_four_dof(
            joint_2=joint_2,
            joint_3=joint_3,
            target_in_arm_frame=target_in_arm_frame_after_rotating_joint_1,
        )

        return Joints(
            j1=joint_1,
            j2=joint_2,
            j3=joint_3,
            j4=joint_4,
            impose_constraints_on_joints=True,
            number_of_joints=self.number_of_joints,
        )

    def _normalize(self, radians: float) -> float:
        """Normalize angle to range [0, 2*pi)"""
        if radians < 0:
            return (2 * math.pi) - abs(radians) % (2 * math.pi)
        elif radians >= 2 * math.pi:
            return radians % (2 * math.pi)
        return radians
