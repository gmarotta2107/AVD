from perception_module.static_object_perception import StaticObjectPerception
from typing import List
import carla


class PerceptionModule:
    def __init__(self, waypoint, world, distance=30):
        self.static_perception = StaticObjectPerception(waypoint, world)
        self.distance = distance

    def update_perception_information(self, waypoint):
        self.static_perception.update(self.distance, waypoint) 

    def get_static(self, name: str, distance =30):
        return self.static_perception.get(name,distance)