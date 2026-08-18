import logging
from logging import Logger
from pathlib import Path

from alitra import Map, MapAlignment, Transform, align_maps
from robot_interface.models.exceptions.robot_exceptions import RobotMapException

from isar_taurob.config import settings


class TaurobMap:
    def __init__(self, map_path: Path) -> None:
        map_alignment: MapAlignment = MapAlignment.from_config(map_path)
        self.map_robot: Map = map_alignment.map_from
        self.map_asset: Map = map_alignment.map_to
        self.name: str = map_alignment.name

        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)

        try:
            self.transform: Transform = align_maps(
                map_alignment.map_from, map_alignment.map_to, rot_axes="z"
            )
        except ValueError:
            error_description: str = (
                f"Failed to align maps using alitra - "
                f"Verify points in map {map_alignment.name}"
            )
            self.logger.exception(error_description)
            raise RobotMapException(error_description)

    def __eq__(self, other):
        return (
            isinstance(other, TaurobMap)
            and self.name == other.name
            and (self.transform.translation == other.transform.translation)
            and self.asset_bounds == other.asset_bounds
        )
