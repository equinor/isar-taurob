import logging
from logging import Logger
from pathlib import Path

from alitra import Bounds, Pose, Position
from robot_interface.models.exceptions.robot_exceptions import RobotMapException

from isar_taurob.config import settings
from isar_taurob.robot.map.taurob_map import TaurobMap
from isar_taurob.robot.taurob_model.taurob_pose import TaurobPose


class MapHandler:
    def __init__(
        self,
        asset_short_name: str = settings.ASSET_SHORT_NAME,
    ) -> None:
        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)

        here = Path(__file__).parent.resolve()
        map_config_folder_path: Path = Path(
            here.parent.parent.joinpath("./config/maps/" + str.lower(asset_short_name))
        )
        self.maps: list[TaurobMap] = [
            TaurobMap(f) for f in map_config_folder_path.glob("*.json")
        ]

        map_names: list[str] = [_map.name for _map in self.maps]
        self.logger.info(f"Loaded maps: {map_names}")
        if self._check_maps_for_overlap(map_names):
            self.logger.error("Loaded maps are overlapping")

        if len(set(map_names)) != len(map_names):
            error_description: str = "One or more maps has duplicate names"
            self.logger.error(error_description)
            raise RobotMapException(error_description=error_description)

        self.last_used_map: TaurobMap | None = None

    @staticmethod
    def _check_if_cubes_overlap(bounds1, bounds2):
        x1_min, y1_min, z1_min = tuple(min(index) for index in zip(*bounds1))
        x1_max, y1_max, z1_max = tuple(max(index) for index in zip(*bounds1))
        x2_min, y2_min, z2_min = tuple(min(index) for index in zip(*bounds2))
        x2_max, y2_max, z2_max = tuple(max(index) for index in zip(*bounds2))

        if x1_min >= x2_max or x2_min >= x1_max:
            return False
        if y1_min >= y2_max or y2_min >= y1_max:
            return False
        if z1_min >= z2_max or z2_min >= z1_max:
            return False
        return True

    def _check_maps_for_overlap(self, map_names: list[str]) -> bool:
        map_bounds: list[Bounds] = []
        for i in range(len(map_names)):
            map_bounds.append(
                (
                    next(filter(lambda x: x.name == map_names[i], self.maps))
                ).map_asset.bounds
            )

        unpack_bounds = [
            (
                (cube.position1.x, cube.position1.y, cube.position1.z),
                (cube.position2.x, cube.position2.y, cube.position2.z),
            )
            for cube in map_bounds
        ]

        if len(unpack_bounds) <= 1:
            return False

        overlapping = []
        for i in range(len(unpack_bounds)):
            for j in range(i + 1, len(unpack_bounds)):
                bounds1 = unpack_bounds[i]
                bounds2 = unpack_bounds[j]
                if self._check_if_cubes_overlap(bounds1, bounds2):
                    overlapping.append((i, j))
        for overlapping_maps in overlapping:
            self.logger.info(
                f"<{map_names[overlapping_maps[0]]}> overlaps with <{map_names[overlapping_maps[1]]}>"
            )

        is_overlapping = len(overlapping) > 0
        return is_overlapping

    def _select_map_by_name(self, name: str) -> TaurobMap:
        try:
            return next(filter(lambda x: x.name == name, self.maps))
        except StopIteration:
            error_description: str = (
                f"The map '{name}' does not exist in the local map storage"
            )
            self.logger.exception(error_description)
            raise RobotMapException(error_description=error_description)

    def select_map_by_asset_position(self, position: Position) -> TaurobMap:
        if position.frame.name == "robot":
            raise RobotMapException(
                "Position is provided in ambiguous frame 'robot'. Recommend transformed to 'asset' frame."
            )

        for _map in self.maps:
            bounds: Bounds = (
                _map.map_robot.bounds
                if _map.map_robot.bounds
                else _map.map_asset.bounds
            )
            transformed_position: Position = _map.transform.transform_position(
                positions=position, from_=position.frame, to_=bounds.frame
            )
            if bounds.position_within_bounds(transformed_position):
                return _map

        raise RobotMapException(f"The position {position} is not within any maps")

    def transform_pose_to_robot(
        self, pose: Pose, recalculate_map: bool = True
    ) -> TaurobPose:
        """
        Find the correct map corresponding to the given pose and transform the pose to
        this map.
        """
        if recalculate_map or self.last_used_map is None:
            _map = self.select_map_by_asset_position(position=pose.position)
            self.last_used_map = _map
        else:
            _map = self.last_used_map
        transformed_pose: Pose = _map.transform.transform_pose(
            pose=pose, from_=pose.frame, to_=_map.map_robot.frame
        )
        return TaurobPose(
            x=transformed_pose.position.x,
            y=transformed_pose.position.y,
            theta=transformed_pose.orientation.to_euler_array(wrap_angles=True)[0],
            map_name=_map.name,
        )

    def transform_position_to_robot(
        self, position: Position, recalculate_map: bool = False
    ) -> Position:
        if recalculate_map or self.last_used_map is None:
            _map = self.select_map_by_asset_position(position=position)
            self.last_used_map = _map
        else:
            _map = self.last_used_map
        return _map.transform.transform_position(
            positions=position, from_=position.frame, to_=_map.map_robot.frame
        )

    def transform_pose_to_asset(self, pose: TaurobPose) -> Pose:
        _map = self._select_map_by_name(name=pose.map_name)
        return _map.transform.transform_pose(
            pose=pose.to_pose(), from_=pose.to_pose().frame, to_=_map.map_asset.frame
        )
