from robot_interface.models.mission.task import TaskStatus


def map_taurob_status_to_task_status(status: str) -> TaskStatus:
    if status == "scheduled":
        return TaskStatus.NotStarted
    elif status == "started":
        return TaskStatus.InProgress
    elif status == "successful":
        return TaskStatus.Successful
    elif status == "canceled":
        return TaskStatus.Cancelled
    elif status == "unsuccessful" or status == "error_unexpected":
        return TaskStatus.Failed
    else:
        return TaskStatus.Failed
