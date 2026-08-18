import pytest
from robot_interface.models.mission.status import TaskStatus

from isar_taurob.robot.api.task_status import map_taurob_status_to_task_status


@pytest.mark.parametrize(
    "status, expected_output",
    [
        ("scheduled", TaskStatus.NotStarted),
        ("started", TaskStatus.InProgress),
        ("successful", TaskStatus.Successful),
        ("cancelled", TaskStatus.Failed),
        ("some_random_string", TaskStatus.Failed),
    ],
)
def test_map_taurob_status_to_task_status(status, expected_output) -> None:
    task_status: TaskStatus = map_taurob_status_to_task_status(status)
    assert task_status is expected_output
