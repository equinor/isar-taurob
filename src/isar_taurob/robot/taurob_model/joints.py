import logging
import math
from dataclasses import dataclass
from logging import Logger

from robot_interface.models.exceptions.robot_exceptions import RobotException

from isar_taurob.config import settings


@dataclass
class Joints:
    def __init__(
        self,
        number_of_joints,
        j1=None,
        j2=None,
        j3=None,
        j4=None,
        j5=None,
        impose_constraints_on_joints=True,
    ) -> None:
        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)

        self.number_of_joints: int = number_of_joints
        self.j1: float | None = j1
        self.j2: float | None = j2
        self.j3: float | None = j3
        self.j4: float | None = j4
        self.j5: float | None = j5
        self.impose_constraints_on_joints: bool = impose_constraints_on_joints
        self.constrainted_joints: list[tuple[str, float]] = []

        if self.number_of_joints == 2:
            self.j1_lower = settings.TWO_DOF_J1_LOWER_CONSTRAINT
            self.j1_upper = settings.TWO_DOF_J1_UPPER_CONSTRAINT
            self.j2_lower = settings.TWO_DOF_J2_LOWER_CONSTRAINT
            self.j2_upper = settings.TWO_DOF_J2_UPPER_CONSTRAINT

        elif self.number_of_joints == 4:
            self.j1_lower = settings.FOUR_DOF_J1_LOWER_CONSTRAINT
            self.j1_upper = settings.FOUR_DOF_J1_UPPER_CONSTRAINT
            self.j2_lower = settings.FOUR_DOF_J2_LOWER_CONSTRAINT
            self.j2_upper = settings.FOUR_DOF_J2_UPPER_CONSTRAINT
            self.j3_lower = settings.FOUR_DOF_J3_LOWER_CONSTRAINT
            self.j3_upper = settings.FOUR_DOF_J3_UPPER_CONSTRAINT
            self.j4_lower = settings.FOUR_DOF_J4_LOWER_CONSTRAINT
            self.j4_upper = settings.FOUR_DOF_J4_UPPER_CONSTRAINT
        else:
            error_description: str = (
                f"An invalid number of joints "
                f"({settings.ROBOT_NUMBER_OF_JOINTS}) was configured"
            )
            self.logger.error(error_description)
            raise RobotException(error_description=error_description)

        if self.impose_constraints_on_joints:
            for joints_field in dir(self):
                if joints_field.startswith("j") and (joints_field[-1].isdigit()):
                    self._check_constraints(joint=joints_field)

    def _check_constraints(self, joint: str):
        if (joint_value := getattr(self, joint)) is None:
            return

        lower_limit = getattr(self, joint + "_lower")
        upper_limit = getattr(self, joint + "_upper")

        assert lower_limit > 0
        assert upper_limit > 0
        assert lower_limit < upper_limit

        blind_zone_width: float = lower_limit + (2 * math.pi - upper_limit)

        if (
            joint_value < lower_limit
            and lower_limit - joint_value < blind_zone_width / 2
            or joint_value > upper_limit
            and joint_value - upper_limit > blind_zone_width / 2
        ):
            setattr(self, joint, lower_limit)
            self.constrainted_joints.append((joint, lower_limit))

        if (
            joint_value < lower_limit
            and lower_limit - joint_value > blind_zone_width / 2
            or joint_value > upper_limit
            and joint_value - upper_limit < blind_zone_width / 2
        ):
            setattr(self, joint, upper_limit)
            self.constrainted_joints.append((joint, upper_limit))
