from dataclasses import dataclass

from isar_taurob.robot.taurob_model.joints import Joints


@dataclass
class ActionTaskTuple:
    mission_instance_id: int | None  # To be set after scheduling. None until then.
    isar_task_id: str
    taurob_action_id: int
    joints: Joints | None
    type: str
