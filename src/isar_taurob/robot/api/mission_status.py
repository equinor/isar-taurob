from robot_interface.models.mission.mission import MissionStatus


def map_taurob_status_to_mission_status(status: str) -> MissionStatus:
    if status == "scheduled":
        return MissionStatus.NotStarted
    elif status == "started":
        return MissionStatus.InProgress
    elif status == "successful":
        return MissionStatus.Successful
    elif status == "canceled":
        return MissionStatus.Cancelled
    elif status == "unsuccessful" or status == "error_unexpected":
        return MissionStatus.Failed
    else:
        return MissionStatus.Failed


def is_mission_completed(status: MissionStatus) -> bool:
    return status in [
        MissionStatus.Successful,
        MissionStatus.Failed,
        MissionStatus.Cancelled,
    ]
