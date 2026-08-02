from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


IGNORED_MODELS = {
    'course_centerline',
    'goal_marker',
    'ground_plane',
    'start_marker',
}


def _pose(element: ET.Element | None) -> tuple[float, float, float, float, float, float]:
    if element is None or not element.text:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    values = [float(value) for value in element.text.split()]
    values.extend([0.0] * (6 - len(values)))
    return tuple(values[:6])


@dataclass(frozen=True)
class OrientedBox:
    name: str
    east: float
    north: float
    altitude: float
    size_east: float
    size_north: float
    size_up: float
    yaw: float

    def signed_distance(
        self,
        east: float,
        north: float,
        altitude: float,
        horizontal_margin: float = 0.0,
        vertical_margin: float = 0.0,
    ) -> float:
        delta_east = east - self.east
        delta_north = north - self.north
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        local_east = cosine * delta_east + sine * delta_north
        local_north = -sine * delta_east + cosine * delta_north

        offsets = (
            abs(local_east) - 0.5 * self.size_east - horizontal_margin,
            abs(local_north) - 0.5 * self.size_north - horizontal_margin,
            abs(altitude - self.altitude) - 0.5 * self.size_up - vertical_margin,
        )
        outside = math.sqrt(sum(max(offset, 0.0) ** 2 for offset in offsets))
        inside = min(max(offsets), 0.0)
        return outside + inside


class ObstacleCourse:
    def __init__(self, boxes: list[OrientedBox]) -> None:
        if not boxes:
            raise ValueError('course contains no collision boxes')
        self.boxes = boxes

    @classmethod
    def from_sdf(cls, path: Path) -> 'ObstacleCourse':
        root = ET.parse(path).getroot()
        world = root.find('world')
        if world is None:
            raise ValueError(f'no world element in {path}')

        boxes = []
        for model in world.findall('model'):
            model_name = model.get('name', 'model')
            if model_name in IGNORED_MODELS:
                continue
            model_pose = _pose(model.find('pose'))
            model_cosine = math.cos(model_pose[5])
            model_sine = math.sin(model_pose[5])
            for link in model.findall('link'):
                link_pose = _pose(link.find('pose'))
                for collision in link.findall('collision'):
                    size_element = collision.find('geometry/box/size')
                    if size_element is None or not size_element.text:
                        continue
                    size = [float(value) for value in size_element.text.split()]
                    if len(size) != 3:
                        continue
                    collision_pose = _pose(collision.find('pose'))
                    local_east = link_pose[0] + collision_pose[0]
                    local_north = link_pose[1] + collision_pose[1]
                    world_east = (
                        model_pose[0] + model_cosine * local_east - model_sine * local_north
                    )
                    world_north = (
                        model_pose[1] + model_sine * local_east + model_cosine * local_north
                    )
                    boxes.append(
                        OrientedBox(
                            name=f'{model_name}/{link.get("name", "link")}',
                            east=world_east,
                            north=world_north,
                            altitude=model_pose[2] + link_pose[2] + collision_pose[2],
                            size_east=size[0],
                            size_north=size[1],
                            size_up=size[2],
                            yaw=model_pose[5] + link_pose[5] + collision_pose[5],
                        )
                    )
        return cls(boxes)

    def clearance(
        self,
        east: float,
        north: float,
        altitude: float,
        vehicle_horizontal_radius: float,
        vehicle_vertical_radius: float,
    ) -> float:
        return min(
            box.signed_distance(
                east,
                north,
                altitude,
                vehicle_horizontal_radius,
                vehicle_vertical_radius,
            )
            for box in self.boxes
        )
