from typing import List
import carla



class StaticObjectPerception:
    """
    Modulo di percezione per oggetti statici (props, semafori, segnali di stop)
    """
    def __init__(self, waypoint: carla.Waypoint, world: carla.World):
        self._waypoint = waypoint
        self._world = world
        self._static_prop_objects: List[carla.Actor] = []
        self._static_traffic_light_objects: List[carla.Actor] = []
        self._stop_sign_objects: List[carla.Actor] = []  
        self._all_static_objects : List[carla.Actor] = []
        self.dirts_street_objects : List[carla.Actor] = []
        self.dirtdebris_street_objects : List[carla.Actor] = []
        self.brokentile_street_objects : List[carla.Actor] = []
        self.id_objects_dict : List[str,List[carla.Actor]] = [("static.prop",self._static_prop_objects),("traffic_light",self._static_traffic_light_objects),("stop",self._stop_sign_objects),("brokentile",self.brokentile_street_objects),("dirtdebris",self.dirtdebris_street_objects),("dirts_street",self.dirts_street_objects)]


    def _distance_to_waypoint(self, actor: carla.Actor) -> float:
        return actor.get_location().distance(self._waypoint.transform.location)

    def update(self, max_distance: float, waypoint: carla.Waypoint = None):
        if waypoint is not None:
            self._waypoint = waypoint
        self.update_all_information(max_distance)

    def update_all_information(self,max_distance):
        self.clear_information()
        self.get_static_actors()
        self.update_actors_list(max_distance)

    def update_actors_list(self, max_distance):
        for actor in self._all_static_objects:
            for id, objects in self.id_objects_dict:
                if self._distance_to_waypoint(actor) < max_distance and "dirtdebris" not in actor.type_id and id in actor.type_id:
                    objects.append(actor)

    def get(self,name: str, distance = 30):
        for id, objects in self.id_objects_dict:
            if name in id:
                return objects 
            
    def get_static_actors(self):
        self._all_static_objects = list(self._world.get_actors())  
    

    def clear_information(self):
        for id, objects in self.id_objects_dict:
            objects.clear()