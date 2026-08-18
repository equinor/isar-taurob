import math

import numpy as np
import pytest
from alitra import Frame, Position

from isar_taurob.config import settings
from isar_taurob.robot.taurob_model.arm import TaurobArm
from isar_taurob.robot.taurob_model.joints import Joints
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose

TWO_DOF_ARM_BASE_X = settings.TWO_DOF_ARM_BASE_X
TWO_DOF_ARM_BASE_Y = settings.TWO_DOF_ARM_BASE_Y
TWO_DOF_ARM_BASE_Z = settings.TWO_DOF_ARM_BASE_Z


def is_position_equal(p1: Position, p2: Position):
    return (
        math.isclose(p1.x, p2.x, abs_tol=1e-09)
        and math.isclose(p1.y, p2.y, abs_tol=1e-09)
        and math.isclose(p1.z, p2.z, abs_tol=1e-09)
    )


mock_taurob_arm = TaurobArm(2)

mock_arm_base_position = mock_taurob_arm._calc_arm_position_in_navigation_frame(
    robot_pose=TaurobPose(x=0, y=0, theta=0, map_name="test_map")
)


mock_arm_base_position_neg45 = mock_taurob_arm._calc_arm_position_in_navigation_frame(
    robot_pose=TaurobPose(x=0, y=0, theta=-math.pi / 4, map_name="test_map")
)


@pytest.mark.parametrize(
    "robot_pose, expected_arm_position",
    [
        (
            TaurobPose(x=0, y=0, theta=0, map_name=""),
            Position(
                x=TWO_DOF_ARM_BASE_X,
                y=TWO_DOF_ARM_BASE_Y,
                z=TWO_DOF_ARM_BASE_Z,
                frame="arm",
            ),
        ),
        (
            TaurobPose(x=0, y=0, theta=math.pi / 2, map_name=""),
            Position(
                x=-TWO_DOF_ARM_BASE_Y,
                y=TWO_DOF_ARM_BASE_X,
                z=TWO_DOF_ARM_BASE_Z,
                frame="arm",
            ),
        ),
        (
            TaurobPose(x=0, y=0, theta=math.pi, map_name=""),
            Position(
                x=-TWO_DOF_ARM_BASE_X,
                y=-TWO_DOF_ARM_BASE_Y,
                z=TWO_DOF_ARM_BASE_Z,
                frame="arm",
            ),
        ),
        (
            TaurobPose(x=0, y=0, theta=-math.pi / 2, map_name=""),
            Position(
                x=TWO_DOF_ARM_BASE_Y,
                y=-TWO_DOF_ARM_BASE_X,
                z=TWO_DOF_ARM_BASE_Z,
                frame="arm",
            ),
        ),
        (
            TaurobPose(x=1, y=0, theta=-math.pi / 2, map_name=""),
            Position(
                x=1 + TWO_DOF_ARM_BASE_Y,
                y=-TWO_DOF_ARM_BASE_X,
                z=TWO_DOF_ARM_BASE_Z,
                frame="arm",
            ),
        ),
        (
            TaurobPose(x=0, y=1, theta=math.pi, map_name=""),
            Position(
                x=-TWO_DOF_ARM_BASE_X,
                y=1 - TWO_DOF_ARM_BASE_Y,
                z=TWO_DOF_ARM_BASE_Z,
                frame="arm",
            ),
        ),
    ],
)
def test_calc_arm_position_in_navigation_frame(
    robot_pose, expected_arm_position, taurob_arm_2_joints: TaurobArm
):
    arm_in_navigation_frame = (
        taurob_arm_2_joints._calc_arm_position_in_navigation_frame(
            robot_pose=robot_pose
        )
    )

    assert is_position_equal(p1=arm_in_navigation_frame, p2=expected_arm_position)


@pytest.mark.parametrize(
    "robot_pose, target_nav_frame, expected_target_arm_frame",
    [
        (
            TaurobPose(
                x=-TWO_DOF_ARM_BASE_X, y=-TWO_DOF_ARM_BASE_Y, theta=0, map_name=""
            ),
            Position(x=1, y=1, z=TWO_DOF_ARM_BASE_Z, frame="nav"),
            Position(x=1, y=1, z=0, frame="arm"),
        ),
        (
            TaurobPose(x=0, y=0, theta=math.pi / 2, map_name=""),
            Position(
                x=-TWO_DOF_ARM_BASE_Y + 1,
                y=TWO_DOF_ARM_BASE_X,
                z=TWO_DOF_ARM_BASE_Z,
                frame="nav",
            ),
            Position(x=0, y=-1, z=0, frame="arm"),
        ),
        (
            TaurobPose(x=0, y=0, theta=math.pi, map_name=""),
            Position(
                x=-TWO_DOF_ARM_BASE_X + 1,
                y=-TWO_DOF_ARM_BASE_Y,
                z=TWO_DOF_ARM_BASE_Z,
                frame="nav",
            ),
            Position(x=-1, y=0, z=0, frame="arm"),
        ),
        (
            TaurobPose(x=1, y=2, theta=-math.pi / 2, map_name=""),
            Position(
                x=1 + TWO_DOF_ARM_BASE_Y + 30,
                y=2 - TWO_DOF_ARM_BASE_X + 40,
                z=TWO_DOF_ARM_BASE_Z + 50,
                frame="nav",
            ),
            Position(x=-40, y=30, z=50, frame="arm"),
        ),
    ],
)
def test_calc_point_in_arm_frame(
    robot_pose,
    target_nav_frame: Position,
    expected_target_arm_frame: Position,
    taurob_arm_2_joints: TaurobArm,
):
    target_arm_frame = taurob_arm_2_joints._calc_point_in_arm_frame(
        robot_pose=robot_pose, point_in_navigation_frame=target_nav_frame
    )

    assert is_position_equal(target_arm_frame, expected_target_arm_frame)


@pytest.mark.parametrize(
    "current_pose, target, expected_joints",
    [
        (
            TaurobPose(x=0, y=0, theta=math.pi / 2, map_name="test_map"),
            Position(
                x=-settings.TWO_DOF_ARM_BASE_Y + settings.TWO_DOF_ARM_LINK_1,
                y=2,
                z=settings.TWO_DOF_ARM_BASE_Z + settings.TWO_DOF_ARM_LINK_2,
                frame=Frame("robot"),
            ),
            Joints(
                j1=mock_taurob_arm.joint_1_at_zero,
                j2=mock_taurob_arm.joint_2_at_vertical_arm,
                number_of_joints=2,
            ),
        ),
        (
            TaurobPose(x=0, y=0, theta=0, map_name="test_map"),
            Position(
                x=mock_arm_base_position.x
                + 2
                + settings.TWO_DOF_ARM_LINK_1 * math.cos(math.pi / 4),
                y=mock_arm_base_position.y
                + 2
                - settings.TWO_DOF_ARM_LINK_1 * math.sin(math.pi / 4),
                z=mock_arm_base_position.z + settings.TWO_DOF_ARM_LINK_2,
                frame=Frame("robot"),
            ),
            Joints(
                j1=mock_taurob_arm.joint_1_at_zero + math.pi / 4,
                j2=mock_taurob_arm.joint_2_at_vertical_arm,
                number_of_joints=2,
            ),
        ),
        (
            TaurobPose(x=0, y=0, theta=-math.pi / 4, map_name="test_map"),
            Position(
                x=mock_arm_base_position_neg45.x
                + 2
                - settings.TWO_DOF_ARM_LINK_1 * math.cos(math.pi / 4),
                y=mock_arm_base_position_neg45.y
                - 2
                - settings.TWO_DOF_ARM_LINK_1 * math.sin(math.pi / 4),
                z=mock_arm_base_position_neg45.z + settings.TWO_DOF_ARM_LINK_2,
                frame=Frame("robot"),
            ),
            Joints(
                j1=mock_taurob_arm.joint_1_at_zero,
                j2=mock_taurob_arm.joint_2_at_vertical_arm,
                number_of_joints=2,
            ),
        ),
        (
            TaurobPose(x=0, y=0, theta=0, map_name="test_map"),
            Position(
                x=100000,
                y=-100000,
                z=settings.TWO_DOF_ARM_BASE_Z + settings.TWO_DOF_ARM_LINK_2,
                frame=Frame("robot"),
            ),
            Joints(
                j1=mock_taurob_arm.joint_1_at_zero - math.pi / 4,
                j2=mock_taurob_arm.joint_2_at_vertical_arm,
                number_of_joints=2,
            ),
        ),
        (
            TaurobPose(x=0, y=0, theta=0, map_name="test_map"),
            Position(
                x=settings.TWO_DOF_ARM_BASE_X
                + settings.TWO_DOF_ARM_LINK_2 * math.sqrt(2),
                y=settings.TWO_DOF_ARM_BASE_Y - settings.TWO_DOF_ARM_LINK_1,
                z=settings.TWO_DOF_ARM_BASE_Z,
                frame=Frame("robot"),
            ),
            Joints(
                j1=mock_taurob_arm.joint_1_at_zero,
                j2=mock_taurob_arm.joint_2_at_vertical_arm - math.pi / 4,
                number_of_joints=2,
            ),
        ),
    ],
)
def test_inverse_kinematics_two_dof(
    current_pose, target, expected_joints, taurob_arm_2_joints: TaurobArm
):
    joints: Joints = taurob_arm_2_joints._solve_inverse_kinematics_two_dof(
        robot_pose=current_pose, target_in_navigation_frame=target
    )

    assert math.isclose(joints.j1, expected_joints.j1, abs_tol=1e-05)
    assert math.isclose(joints.j2, expected_joints.j2, abs_tol=1e-05)


@pytest.mark.parametrize(
    "target_in_arm_frame, expected_joint_1",
    [
        (Position(x=2, y=0, z=0, frame=Frame("robot")), 2.01),  # 0°
        (Position(x=2, y=2, z=0, frame=Frame("robot")), 2.80),  # 45°
        (Position(x=0, y=2, z=0, frame=Frame("robot")), 3.58),  # 90°
        (Position(x=-2, y=2, z=0, frame=Frame("robot")), 4.37),  # 135°
        (Position(x=-2, y=0, z=0, frame=Frame("robot")), 5.15),  # 180°
        (
            Position(x=-2, y=-2, z=0, frame=Frame("robot")),
            5.93,
        ),  # 225° (in blind zone)
        (Position(x=0, y=-2, z=0, frame=Frame("robot")), 0.44),  # 270°
        (Position(x=2, y=-2, z=0, frame=Frame("robot")), 1.22),  # 315°
    ],
)
def test_solve_inverse_kinematics_joint_1_four_dof(
    target_in_arm_frame, expected_joint_1, taurob_arm_2_joints: TaurobArm
):
    joint_1 = taurob_arm_2_joints._solve_inverse_kinematics_joint_1_four_dof(
        target_in_arm_frame
    )
    assert math.isclose(joint_1, expected_joint_1, abs_tol=1e-02)


def test_basic_logic_inverse_kinematics_four_dof(taurob_arm_4_joints: TaurobArm):
    pose = TaurobPose(x=0, y=0, theta=0, map_name="test_map")
    target_1 = Position(
        x=0,
        y=100000,
        z=0,
        frame=Frame("robot"),
    )
    target_2 = Position(
        x=100000,
        y=0,
        z=0,
        frame=Frame("robot"),
    )
    target_3 = Position(
        x=100000 / math.sqrt(2),
        y=100000 / math.sqrt(2),
        z=0,
        frame=Frame("robot"),
    )

    joints_case1 = taurob_arm_4_joints._solve_inverse_kinematics_four_dof(
        robot_pose=pose, target_in_navigation_frame=target_1
    )
    joints_case2 = taurob_arm_4_joints._solve_inverse_kinematics_four_dof(
        robot_pose=pose, target_in_navigation_frame=target_2
    )
    joints_case3 = taurob_arm_4_joints._solve_inverse_kinematics_four_dof(
        robot_pose=pose, target_in_navigation_frame=target_3
    )

    assert math.isclose(joints_case1.j4, joints_case2.j4, abs_tol=1e-06)
    assert math.isclose(joints_case1.j4, joints_case3.j4, abs_tol=1e-06)

    assert math.isclose(
        abs(joints_case1.j1 - joints_case2.j1), math.pi / 2, abs_tol=1e-03
    )
    assert math.isclose(
        abs(joints_case1.j1 - joints_case3.j1), math.pi / 4, abs_tol=1e-03
    )


@pytest.mark.parametrize(
    "target_in_arm_frame, joint_1, expected_target_in_arm_frame",
    [
        (
            Position(x=3, y=0, z=0, frame="arm"),
            settings.FOUR_DOF_JOINT_1_AT_ZERO_POSITION,
            Position(x=3, y=0, z=0, frame="arm"),
        ),
        (
            Position(x=0, y=3, z=0, frame="arm"),
            settings.FOUR_DOF_JOINT_1_AT_ZERO_POSITION + math.pi / 2,
            Position(x=3, y=0, z=0, frame="arm"),
        ),
        (
            Position(x=0, y=-3, z=0, frame="arm"),
            settings.FOUR_DOF_JOINT_1_AT_ZERO_POSITION - math.pi / 2,
            Position(x=3, y=0, z=0, frame="arm"),
        ),
        (
            Position(x=3, y=3, z=0, frame="arm"),
            settings.FOUR_DOF_JOINT_1_AT_ZERO_POSITION + math.pi / 4,
            Position(x=4.2426, y=0, z=0, frame="arm"),
        ),
        (
            Position(x=-2, y=-5, z=0, frame="arm"),
            settings.FOUR_DOF_JOINT_1_AT_ZERO_POSITION - math.pi / 2 - 0.3805,
            Position(x=5.3852, y=0, z=0, frame="arm"),
        ),
    ],
)
def test_calculate_target_in_arm_frame_after_rotating_joint_1(
    target_in_arm_frame: Position,
    joint_1: float,
    expected_target_in_arm_frame: Position,
    taurob_arm_2_joints: TaurobArm,
):
    target_in_arm_frame_after_rotating_joint_1: Position = (
        taurob_arm_2_joints._calculate_target_in_arm_frame_after_rotating_joint_1(
            joint_1=joint_1, target_in_arm_frame=target_in_arm_frame
        )
    )

    assert math.isclose(
        abs(
            target_in_arm_frame_after_rotating_joint_1.to_array()[0]
            - expected_target_in_arm_frame.to_array()[0]
        ),
        0,
        abs_tol=1e-03,
    )
    assert math.isclose(
        abs(target_in_arm_frame_after_rotating_joint_1.to_array()[1]), 0, abs_tol=1e-03
    )
    assert math.isclose(
        abs(
            target_in_arm_frame_after_rotating_joint_1.to_array()[2]
            - expected_target_in_arm_frame.to_array()[2]
        ),
        0,
        abs_tol=1e-03,
    )


@pytest.mark.parametrize(
    "joint_2, joint_3, expected_joint_4_pos_in_arm_frame",
    [
        (2.637, 1.701, Position(x=0.13214, y=0, z=0.69573, frame="arm")),
    ],
)
def test_calculate_joint_4_pos_in_arm_frame(
    joint_2: float,
    joint_3: float,
    expected_joint_4_pos_in_arm_frame: Position,
    taurob_arm_4_joints: TaurobArm,
):
    joint_4_pos_in_arm_frame: Position = (
        taurob_arm_4_joints._calculate_joint_4_pos_in_arm_frame(
            joint_2=joint_2, joint_3=joint_3
        )
    )

    assert math.isclose(
        abs(
            joint_4_pos_in_arm_frame.to_array()[0]
            - expected_joint_4_pos_in_arm_frame.to_array()[0]
        ),
        0,
        abs_tol=1e-03,
    )
    assert math.isclose(
        abs(
            joint_4_pos_in_arm_frame.to_array()[1]
            - expected_joint_4_pos_in_arm_frame.to_array()[1]
        ),
        0,
        abs_tol=1e-03,
    )
    assert math.isclose(
        abs(
            joint_4_pos_in_arm_frame.to_array()[2]
            - expected_joint_4_pos_in_arm_frame.to_array()[2]
        ),
        0,
        abs_tol=1e-03,
    )


@pytest.mark.parametrize(
    "target_in_arm_frame, joint_4_pos_in_arm_frame, expected_vector",
    [
        (
            Position(x=0, y=0, z=0, frame="arm"),
            Position(x=0, y=0, z=0, frame="arm"),
            np.array([0.0, 0.0, 0.0]),
        ),
        (
            Position(x=5, y=3, z=4, frame="arm"),
            Position(x=0.5, y=0.5, z=1, frame="arm"),
            np.array([4.5, 2.5, 3]),
        ),
    ],
)
def test_calculate_vector_from_joint_4_to_target_in_arm_frame(
    target_in_arm_frame: Position,
    joint_4_pos_in_arm_frame: Position,
    expected_vector: np.ndarray,
    taurob_arm_4_joints: TaurobArm,
):
    vector_from_joint_4_to_target_in_arm_frame: np.ndarray = (
        taurob_arm_4_joints._calculate_vector_from_joint_4_to_target_in_arm_frame(
            target_in_arm_frame=target_in_arm_frame,
            joint_4_pos_in_arm_frame=joint_4_pos_in_arm_frame,
        )
    )
    assert (vector_from_joint_4_to_target_in_arm_frame == expected_vector).all()


@pytest.mark.parametrize(
    "target_in_arm_frame, joint_4_pos_in_arm_frame, expected_angle_between_link_4_and_arm_frame_x_axis",
    [
        (
            Position(x=5.13214, y=0.0, z=0.69573, frame="arm"),
            Position(x=0.13214, y=0.0, z=0.69573, frame="arm"),
            1.5508,
        ),
        (
            Position(x=5.13214, y=0.0, z=5.69573, frame="arm"),
            Position(x=0.13214, y=0.0, z=0.69573, frame="arm"),
            1.5566 + math.pi / 4,
        ),
        (
            Position(x=5.13214, y=0.0, z=0.19573, frame="arm"),
            Position(x=0.13214, y=0.0, z=0.69573, frame="arm"),
            1.4511,
        ),
        (
            Position(x=0.63214, y=0.0, z=0.69573, frame="arm"),
            Position(x=0.13214, y=0.0, z=0.69573, frame="arm"),
            1.3694,
        ),
    ],
)
def test_calculate_angle_between_link_4_and_arm_frame_x_axis(
    target_in_arm_frame: Position,
    joint_4_pos_in_arm_frame: Position,
    expected_angle_between_link_4_and_arm_frame_x_axis: float,
    taurob_arm_4_joints: TaurobArm,
):
    vector_from_joint_4_to_target_in_arm_frame: np.ndarray = (
        taurob_arm_4_joints._calculate_vector_from_joint_4_to_target_in_arm_frame(
            target_in_arm_frame=target_in_arm_frame,
            joint_4_pos_in_arm_frame=joint_4_pos_in_arm_frame,
        )
    )

    angle_between_link_4_and_arm_frame_x_axis: float = (
        taurob_arm_4_joints._calculate_angle_between_link_4_and_arm_frame_x_axis(
            vector_from_joint_4_to_target_in_arm_frame=vector_from_joint_4_to_target_in_arm_frame
        )
    )

    print(
        f"Difference: {abs(angle_between_link_4_and_arm_frame_x_axis - expected_angle_between_link_4_and_arm_frame_x_axis)}"
    )

    assert math.isclose(
        abs(
            angle_between_link_4_and_arm_frame_x_axis
            - expected_angle_between_link_4_and_arm_frame_x_axis
        ),
        0.0,
        abs_tol=1e-03,
    )


@pytest.mark.parametrize(
    "joint_2, joint_3, target_in_arm_frame, expected_joint_4",
    [
        (
            2.637,
            1.701,
            Position(x=5.13214, y=0.0, z=0.69573, frame="arm"),
            2.3472 + settings.FOUR_DOF_JOINT_4_AT_ZERO_POSITION,
        ),
        (
            2.637,
            1.701,
            Position(x=5.13214, y=0.0, z=5.69573, frame="arm"),
            1.5564 + settings.FOUR_DOF_JOINT_4_AT_ZERO_POSITION,
        ),
        (
            2.637,
            1.701,
            Position(x=5.13214, y=0.0, z=0.19573, frame="arm"),
            2.4469 + settings.FOUR_DOF_JOINT_4_AT_ZERO_POSITION,
        ),
        (
            2.637,
            1.701,
            Position(x=0.63214, y=0.0, z=0.69573, frame="arm"),
            2.5286 + settings.FOUR_DOF_JOINT_4_AT_ZERO_POSITION,
        ),
    ],
)
def test_solve_inverse_kinematics_joint_4_four_dof(
    joint_2: float,
    joint_3: float,
    target_in_arm_frame: Position,
    expected_joint_4: float,
    taurob_arm_4_joints: TaurobArm,
):
    joint_4: float = taurob_arm_4_joints._solve_inverse_kinematics_joint_4_four_dof(
        joint_2=joint_2, joint_3=joint_3, target_in_arm_frame=target_in_arm_frame
    )

    assert math.isclose(abs(joint_4 - expected_joint_4), 0.0, abs_tol=1e-03)


@pytest.mark.parametrize(
    "angle, normalized_angle",
    [
        (0, 0),
        (2 * math.pi, 0),
        (-1 / 2 * math.pi, 3 / 2 * math.pi),
        (5 / 2 * math.pi, 1 / 2 * math.pi),
        (9 / 2 * math.pi, 1 / 2 * math.pi),
    ],
)
def test_normalize(angle, normalized_angle, taurob_arm_2_joints: TaurobArm):
    assert taurob_arm_2_joints._normalize(radians=angle) == normalized_angle
