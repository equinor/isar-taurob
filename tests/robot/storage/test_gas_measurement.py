from datetime import UTC, datetime

from pytest_mock import MockerFixture
from robot_interface.models.inspection.inspection import GasMeasurement
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


def test_get_inspection_for_gas_measurement(
    robot: Robot,
    mocker: MockerFixture,
) -> None:
    gas_task = TakeXam8000GasMeasurement(
        id="test_id",
        robot_pose=mock_pose(),
    )

    robot.action_task_map[gas_task.id] = ActionTaskTuple(
        mission_instance_id=123,
        isar_task_id=gas_task.id,
        taurob_action_id=1234,
    )
    robot.task_id_to_completion_pose[gas_task.id] = mock_taurob_pose()
    robot.task_id_to_completion_time_stamp[gas_task.id] = datetime.now(UTC)

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
