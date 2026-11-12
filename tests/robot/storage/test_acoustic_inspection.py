import json
from datetime import UTC, datetime
from pathlib import Path
from unittest.mock import Mock, call

import pytest
from isar.models.mqtt_queue import MQTTQueue
from isar.storage.storage_interface import BlobStoragePath
from isar.storage.uploader import _publish_inspection_result
from pytest_mock import MockerFixture
from requests import Response
from robot_interface.models.exceptions.robot_exceptions import (
    RobotRetrieveInspectionException,
)
from robot_interface.models.inspection.inspection import (
    AcousticMeasurement,
    InspectionMetadata,
)
from robot_interface.models.mission.mission import Mission
from robot_interface.models.mission.task import TakeAcousticMeasurement

from isar_taurob import Robot
from isar_taurob.robot.api.acoustic_metadata import (
    ACOUSTIC_MEASUREMENT_DURATION,
    AcousticMetadata,
)
from isar_taurob.robot.storage.storage import Storage
from isar_taurob.robot.taurob_model.inspection_tuple import ActionTaskTuple
from tests.utilities.mock_api_return_values.mock_taurob_api import MockTaurobAPI
from tests.utilities.mock_models.mock_robot_variables import mock_taurob_pose

DATA_DIR = Path(__file__).parents[2] / "test_data"


@pytest.fixture()
def context(acoustic_task: TakeAcousticMeasurement) -> InspectionMetadata:
    return InspectionMetadata(
        start_time=datetime.now(UTC),
        robot_pose=acoustic_task.robot_pose,
        target_position=acoustic_task.target,
        file_type="",
        tag_id=acoustic_task.tag_id,
        inspection_description=acoustic_task.inspection_description,
    )


@pytest.mark.parametrize("reverse_artifacts", [False, True])
def test_robot_returns_acoustic_video_and_publishes_metadata(
    robot: Robot,
    acoustic_task: TakeAcousticMeasurement,
    mocker: MockerFixture,
    reverse_artifacts: bool,
) -> None:
    assert (
        robot.scheduler.get_measurement_metadata(acoustic_task).duration
        == ACOUSTIC_MEASUREMENT_DURATION
    )
    robot.action_task_map[acoustic_task.id] = ActionTaskTuple(
        mission_instance_id=39544,
        isar_task_id=acoustic_task.id,
        taurob_action_id=1234,
    )
    robot.task_id_to_completion_pose[acoustic_task.id] = mock_taurob_pose()
    robot.task_id_to_completion_time_stamp[acoustic_task.id] = datetime.now(UTC)
    details = MockTaurobAPI.get_direct_acoustic_measurement_details()
    # Use the real artifact list with a scheduled mission's step association.
    details[0]["mission_step"] = {"step_id": 1234, "step_type": "measure"}
    if reverse_artifacts:
        details[0]["measurement_results"].reverse()
    mocker.patch.object(
        robot.storage.robot_api,
        "get_mission_instance_details",
        return_value=details,
    )
    base_url = "https://127.0.0.1:8443/basic/fleet/measurement"
    responses = {
        f"{base_url}/18306": Mock(
            spec=Response, content=(DATA_DIR / "acoustic_metadata_v1.json").read_bytes()
        ),
        f"{base_url}/18307": Mock(
            spec=Response, content=(DATA_DIR / "acoustic_metadata_v2.json").read_bytes()
        ),
        f"{base_url}/18308": Mock(spec=Response, content=b"original acoustic video"),
    }
    get_data = mocker.patch.object(
        robot.storage.robot_api,
        "get_url_data",
        side_effect=responses.__getitem__,
    )
    normalize = mocker.patch.object(
        robot.storage, "_normalize_video_to_constant_frame_rate"
    )

    inspections = robot.get_inspections(acoustic_task)
    inspection = inspections[0]

    assert isinstance(inspection, AcousticMeasurement)

    assert inspection.id == acoustic_task.id
    assert inspection.data == b"original acoustic video"
    normalize.assert_not_called()
    metadata_calls = [call(f"{base_url}/18306"), call(f"{base_url}/18307")]
    if reverse_artifacts:
        metadata_calls.reverse()
    assert get_data.call_args_list == [*metadata_calls, call(f"{base_url}/18308")]
    metadata = inspection.metadata
    assert metadata.start_time == datetime(2026, 10, 8, 8, 55, 28, tzinfo=UTC)
    assert metadata.duration == ACOUSTIC_MEASUREMENT_DURATION
    assert metadata.file_type == "mp4"
    assert metadata.snr_value == 81.12086486816406
    assert metadata.leak_rate == 9.248553276062012
    assert metadata.leak_rate_unit == "l/min"
    assert metadata.sound_pressure_level_at_source_db == 72.97914123535156
    assert metadata.sound_pressure_level_at_sensor_db is None
    assert metadata.distance_to_source == 2.0
    assert metadata.result == "RI_ANOMALY"
    assert metadata.frequency_from == 33465.484375
    assert metadata.frequency_to == 41321.7109375
    assert metadata.target_position == acoustic_task.target
    assert metadata.robot_pose.frame.name == "asset"
    assert metadata.tag_id == acoustic_task.tag_id
    assert metadata.inspection_description == acoustic_task.inspection_description

    mqtt = MQTTQueue()
    _publish_inspection_result(
        mqtt,
        inspection,
        BlobStoragePath(
            storage_account="test", blob_container="raw", blob_name="a.mp4"
        ),
        Mission(id="mission", name="Acoustic", tasks=[acoustic_task]),
    )
    payload = json.loads(mqtt.get().payload)
    assert payload["inspection_type"] == "AcousticMeasurement"
    assert payload["acoustic_metadata"]["sound_pressure_level_at_sensor_db"] is None
    assert payload["acoustic_metadata"]["leak_rate_unit"] == "l/min"
    assert payload["acoustic_metadata"]["result"] == "RI_ANOMALY"
    assert payload["duration"] == ACOUSTIC_MEASUREMENT_DURATION


@pytest.mark.parametrize(
    "snr, expected", [(74.9, "RI_NORMAL"), (75.0, "RI_NORMAL"), (75.1, "RI_ANOMALY")]
)
def test_acoustic_verdict_uses_sensor_threshold(
    snr: float, expected: str, context: InspectionMetadata
) -> None:
    data = json.loads((DATA_DIR / "acoustic_metadata_v1.json").read_text())
    data["snrValue"] = snr
    readings = AcousticMetadata.model_validate(data)

    assert readings.to_inspection_metadata(context).result == expected


@pytest.mark.parametrize(
    "change",
    [
        {"type": "mechanicalInspection"},
        {"snrValue": None},
        {"estimatedDistanceUnit": "feet"},
        {"frequencySelectionTo": 100},
        {"ldMetadata": {"results": {"rate": 9.25}}},
    ],
)
def test_invalid_metadata_raises_robot_error(
    storage: Storage,
    acoustic_task: TakeAcousticMeasurement,
    context: InspectionMetadata,
    mocker: MockerFixture,
    change: dict,
) -> None:
    data = json.loads((DATA_DIR / "acoustic_metadata_v1.json").read_text())
    data.update(change)
    mocker.patch.object(
        storage.robot_api,
        "get_mission_instance_details",
        return_value=MockTaurobAPI.get_acoustic_mission_details(),
    )
    get_data = mocker.patch.object(
        storage.robot_api,
        "get_url_data",
        return_value=Mock(spec=Response, content=json.dumps(data).encode()),
    )

    with pytest.raises(RobotRetrieveInspectionException) as error:
        storage.get_inspections(126, context, 1234, acoustic_task)

    assert "Invalid acoustic metadata" in error.value.error_description
    get_data.assert_called_once_with("https://robot/metadata")


@pytest.mark.parametrize("version", [2, 3])
def test_no_fallback_when_version1_is_missing(
    storage: Storage,
    context: InspectionMetadata,
    acoustic_task: TakeAcousticMeasurement,
    mocker: MockerFixture,
    version: int,
) -> None:
    data = json.loads((DATA_DIR / "acoustic_metadata_v2.json").read_text())
    data["version"] = version
    mocker.patch.object(
        storage.robot_api,
        "get_mission_instance_details",
        return_value=MockTaurobAPI.get_acoustic_mission_details(),
    )
    get_data = mocker.patch.object(
        storage.robot_api,
        "get_url_data",
        return_value=Mock(spec=Response, content=json.dumps(data).encode()),
    )

    with pytest.raises(RobotRetrieveInspectionException) as error:
        storage.get_inspections(126, context, 1234, acoustic_task)

    assert "one version 1 metadata file" in error.value.error_description
    get_data.assert_called_once_with("https://robot/metadata")


@pytest.mark.parametrize(
    "media_type, count", [("video/mp4", 0), ("video/mp4", 2), ("application/json", 0)]
)
def test_missing_or_ambiguous_artifacts_raise_robot_error(
    storage: Storage,
    context: InspectionMetadata,
    acoustic_task: TakeAcousticMeasurement,
    mocker: MockerFixture,
    media_type: str,
    count: int,
) -> None:
    details = MockTaurobAPI.get_acoustic_mission_details()
    artifacts = details[0]["measurement_results"]
    matching = next(item for item in artifacts if item["media_type"] == media_type)
    artifacts = [item for item in artifacts if item["media_type"] != media_type] + [
        matching
    ] * count
    details[0]["measurement_results"] = artifacts
    mocker.patch.object(
        storage.robot_api, "get_mission_instance_details", return_value=details
    )
    mocker.patch.object(
        storage.robot_api,
        "get_url_data",
        return_value=Mock(
            spec=Response, content=(DATA_DIR / "acoustic_metadata_v1.json").read_bytes()
        ),
    )

    with pytest.raises(RobotRetrieveInspectionException) as error:
        storage.get_inspections(126, context, 1234, acoustic_task)
    assert "one version 1 metadata file and one video" in error.value.error_description
