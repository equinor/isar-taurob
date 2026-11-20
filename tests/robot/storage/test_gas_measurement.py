from datetime import UTC, datetime

import pytest
from pytest_mock import MockerFixture
from robot_interface.models.exceptions.robot_exceptions import (
    RobotRetrieveInspectionException,
)
from robot_interface.models.inspection.inspection import GasMeasurement, O2Measurement
from robot_interface.models.mission.task import TakeXam8000GasMeasurement

from isar_taurob import Robot
from isar_taurob.robot.api.sensor import SensorValue
from isar_taurob.robot.taurob_model.inspection_tuple import ActionTaskTuple
from tests.utilities.mock_api_return_values.mock_taurob_api import MockTaurobAPI
from tests.utilities.mock_models.mock_robot_variables import mock_pose, mock_taurob_pose


def _sensor_snapshot() -> dict[str, SensorValue]:
    return {
        entry["name"]: SensorValue.model_validate(entry)
        for entry in MockTaurobAPI.get_all_sensor_values()
    }


@pytest.fixture
def gas_task(robot: Robot) -> TakeXam8000GasMeasurement:
    task = TakeXam8000GasMeasurement(
        id="test_id",
        robot_pose=mock_pose(),
    )

    robot.action_task_map[task.id] = ActionTaskTuple(
        mission_instance_id=123,
        isar_task_id=task.id,
        taurob_action_id=1234,
    )
    robot.task_id_to_completion_pose[task.id] = mock_taurob_pose()
    robot.task_id_to_completion_time_stamp[task.id] = datetime.now(UTC)
    return task


def test_get_inspection_for_gas_measurement(
    robot: Robot,
    gas_task: TakeXam8000GasMeasurement,
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(
        robot.telemetry.robot_api,
        "get_all_sensor_values",
        return_value=_sensor_snapshot(),
    )

    inspections = robot.get_inspections(gas_task)

    assert all(isinstance(inspection, GasMeasurement) for inspection in inspections)
    assert len(inspections) == 6
    assert {
        type(inspection).__name__: inspection.decimal_places
        for inspection in inspections
    } == {
        "IsobutyleneMeasurement": 1,
        "CO2Measurement": 1,
        "TolueneMeasurement": 1,
        "O2Measurement": 2,
        "COMeasurement": 1,
        "H2SMeasurement": 1,
    }


def test_get_gas_measurement_accepts_integer_in_partial_payload(
    robot: Robot, gas_task: TakeXam8000GasMeasurement, mocker: MockerFixture
) -> None:
    mocker.patch.object(
        robot.telemetry,
        "_get_value",
        return_value='{"O2": {"value": 21, "unit": "Vol%"}, "CO2": null, "OTHER": []}',
    )

    (inspection,) = robot.get_inspections(gas_task)

    assert isinstance(inspection, O2Measurement)
    assert inspection.decimal_places == 0


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        ("[]", "Invalid gas measurement"),
        ('{"O2": {"value": "20.90", "unit": "Vol%"}}', r"O2\.value"),
        ('{"O2": {"value": 1e400, "unit": "Vol%"}}', r"O2\.value"),
    ],
)
def test_invalid_gas_measurement_raises_inspection_error(
    robot: Robot,
    gas_task: TakeXam8000GasMeasurement,
    mocker: MockerFixture,
    payload: str,
    message: str,
) -> None:
    mocker.patch.object(robot.telemetry, "_get_value", return_value=payload)

    with pytest.raises(RobotRetrieveInspectionException, match=message) as error:
        robot.get_inspections(gas_task)

    assert gas_task.id in str(error.value)
