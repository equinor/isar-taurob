import math

import pytest

from isar_taurob.robot.taurob_model.joints import Joints

joints = Joints(number_of_joints=2)


@pytest.mark.parametrize(
    "joint_1, joint_2, expected_joint_1, expected_joint_2",
    [
        (
            1.34,
            3.348,
            1.34,
            3.348,
        ),
        (
            joints.j1_lower - 0.1,
            joints.j2_lower - 0.1,
            joints.j1_lower,
            joints.j2_lower,
        ),
        (
            joints.j1_upper + 0.1,
            joints.j2_upper + 0.1,
            joints.j1_upper,
            joints.j2_upper,
        ),
        (
            joints.j1_upper
            + (joints.j1_lower + (2 * math.pi - joints.j1_upper)) / 2
            + 0.01,
            joints.j2_upper
            + (joints.j2_lower + (2 * math.pi - joints.j2_upper)) / 2
            - 0.01,
            joints.j1_lower,
            joints.j2_upper,
        ),
    ],
)
def test_joint_constraints(joint_1, joint_2, expected_joint_1, expected_joint_2):
    joints = Joints(
        j1=joint_1, j2=joint_2, number_of_joints=2, impose_constraints_on_joints=True
    )
    assert joints.j1 == expected_joint_1
    assert joints.j2 == expected_joint_2
