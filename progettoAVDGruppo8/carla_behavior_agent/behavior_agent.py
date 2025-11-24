# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations. """

import random
import numpy as np
import carla
from basic_agent import BasicAgent
from local_planner import RoadOption
from behavior_types import Cautious, Aggressive, Normal
from basic_agent import BasicAgent
from misc import get_speed, positive, is_within_distance, compute_distance
from perception_module.perception import PerceptionModule
import math


class BehaviorAgent(BasicAgent):
    """
    BehaviorAgent implements an agent that navigates scenes to reach a given
    target destination, by computing the shortest possible path to it.
    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles. Lane changing
    decisions can be taken by analyzing the surrounding environment such as tailgating avoidance.
    Adding to these are possible behaviors, the agent can also keep safety distance
    from a car in front of it by tracking the instantaneous time to collision
    and keeping it in a certain range. Finally, different sets of behaviors
    are encoded in the agent, from cautious to a more aggressive ones.
    """

    def __init__(self, vehicle, behavior='normal', opt_dict={}, map_inst=None, grp_inst=None):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param behavior: type of agent to apply
        """

        super().__init__(vehicle, opt_dict=opt_dict, map_inst=map_inst, grp_inst=grp_inst)
        self.guarda_passi_avanti = 0

        # Informazioni iniziali
        self._velocita = 0
        self._limite_velocita = 0
        self._direzione = None
        self._velocita_in_curva = 30
        self._direzione_entrante = None
        self._waypoint_entrante = None
        self._velocita_minima = 5
        self._behavior = None
        self._risoluzione_campionamento = 4.5
        self.cambia_corsia_sinistra=False
        
        
        self._bike_type_list = ['vehicle.gazelle.omafiets', 'vehicle.bh.crossbike', 'vehicle.diamondback.century','vehicle.yamaha.yzf','vehicle.vespa.zx125','vehicle.kawasaki.ninja','vehicle.harley-davidson.low_rider']
        self._ignora_stop = None
        self._stop_counter = 0
        self._in_incrocio = False
        self._lista_waypoint_incroci = None
        self._indice_ultimo_incrocio_waypoint = 0
        self.front_ultimo_veicolo = None

        
        if behavior == 'cautious':
            self._behavior = Cautious()

        elif behavior == 'normal':
            self._behavior = Normal()

        elif behavior == 'aggressive':
            self._behavior = Aggressive()

        transform_iniziale = vehicle.get_transform()
        self.percezione = PerceptionModule(self._map.get_waypoint(transform_iniziale.location), self._world, distance=30)
        self._velocità_massima_iniziale = self._behavior.max_speed

    def biker_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        with any biker.

            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a biker nearby, False if not
            :return vehicle: nearby biker
            :return distance: distance to nearby biker
            :return same_lane: if biker is in the same line or not
        """

        lista_bici = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        lista_bici = [v for v in lista_bici if dist(v) < 35 and v.id != self._vehicle.id and v.type_id in self._bike_type_list]

        if len(lista_bici) > 0:
            if self._direzione == RoadOption.CHANGELANELEFT:
                return self._biker_obstacle_detected(
                    lista_bici, max(
                        self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=180, lane_offset=-1)
            elif self._direzione == RoadOption.CHANGELANERIGHT:
                return self._biker_obstacle_detected(
                    lista_bici, max(
                        self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=180, lane_offset=1)
            else:
                return self._biker_obstacle_detected(
                    lista_bici, max(
                        self._behavior.min_proximity_threshold, self._limite_velocita / 3), up_angle_th=135)
        else:
            return [(False, None, -1, None)]
        
    def side_obstacle_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        with any stopped vehicles.

            :param waypoint: current waypoint of the agent
            :return list(carla.Vehicle): sorted list contained all vehicles or a list with a single element that indicates that is not a vehicle 
        """

        lista_veicoli = self._world.get_actors().filter("*vehicle*")

        def dist(v): return v.get_location().distance(waypoint.transform.location)
        temp_lista_veicoli = [v for v in lista_veicoli if dist(v) < 50]

        ego_transform = self._vehicle.get_transform()
        ego_waypoint = self._map.get_waypoint(self._vehicle.get_location())


        if waypoint.is_junction or self._waypoint_entrante.is_junction:
            return []

        waypoint_corsia_destra = waypoint.get_right_lane()
        id_corsia_destra = waypoint_corsia_destra.lane_id

        # Ottieni la trasformazione della parte anteriore del veicolo ego
        ego_vettore_forward = ego_transform.get_forward_vector()
        ego_extent = self._vehicle.bounding_box.extent.x
        ego_front_transform = ego_transform
        ego_front_transform.location += carla.Location(
            x=ego_extent * ego_vettore_forward.x,
            y=ego_extent * ego_vettore_forward.y,
        )

        lista_veicoli = []  
        for vehicle in temp_lista_veicoli:
            transform_veicolo = vehicle.get_transform()
            waypoint_veicolo = self._map.get_waypoint(transform_veicolo.location, lane_type=carla.LaneType.Any)

            # verifica se almeno uno dei vertici del bounding box si trova nello stesso road id Del veicolo ego
            bounding_box_veicolo_target = vehicle.bounding_box.get_world_vertices(transform_veicolo)
            nella_stessa_corsia_id = False
            for point_location in bounding_box_veicolo_target:
                if (self._map.get_waypoint(point_location, lane_type=carla.LaneType.Any).lane_id == (ego_waypoint.lane_id)):
                    nella_stessa_corsia_id = True
                    break

            if waypoint_veicolo.road_id == waypoint.road_id and waypoint_veicolo.lane_id == id_corsia_destra \
                and is_within_distance(transform_veicolo, ego_front_transform, max_distance=40, angle_interval=[0,45]) \
                and waypoint_veicolo.lane_type == carla.LaneType.Shoulder and nella_stessa_corsia_id:
                lista_veicoli.append((True, vehicle, compute_distance(transform_veicolo.location, ego_transform.location)))
        
        lista_veicoli_ordinata = sorted(lista_veicoli, key=lambda x: x[2])

        return lista_veicoli_ordinata
    
    def get_actor_moving_direction(self, target_transform, reference_transform):
        """
        Method to determine the moving direction of an actor relative to the reference vehicle.

        :param target_transform: The transform of the target actor.
        :param reference_transform: The transform of the reference (ego) vehicle.

        :return: The direction in which the target actor is moving relative to the reference vehicle. 
                Possible values are 'forward' or 'crossing'.
        """
        ego_forward = reference_transform.get_forward_vector()
        vettori_ego_forward = np.array([ego_forward.x, ego_forward.y])

        attore_forward = target_transform.get_forward_vector()
        vettore_attore_forward = np.array([attore_forward.x, attore_forward.y])

        norm_target = np.linalg.norm(vettore_attore_forward)

        angolo = math.degrees(math.acos(np.clip(np.dot(vettori_ego_forward, vettore_attore_forward) / norm_target, -1., 1.)))

        direzione = "forward"
        if not (0 < angolo < 60):
            direzione = "crossing"

        return direzione


    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        """
        self._update_information()

        waypoint_attuale = self._map.get_waypoint(self._vehicle.get_location())
        self.percezione.update_perception_information(waypoint_attuale)

        # Ottieni solo gli attori statici (già filtrati)
        attori_statici_filtrati = self.percezione.get_static("static.prop")

        
        # gestione velocità massima in base a condizione atmosferica
        precipitazioni = self._world.get_weather().precipitation_deposits
        self._behavior.max_speed = self._velocità_massima_iniziale - 15 * (precipitazioni/100)


        controllo = None
        if self._behavior.tailgate_counter > 0:
            self._behavior.tailgate_counter -= 1

        ego_posizione_veicolo = self._vehicle.get_location()
        ego_waypoint_veicolo = self._map.get_waypoint(ego_posizione_veicolo)
        
        influenzato_dal_segnale_di_stop, distanza_segnale_stop = self.stop_sign_manager()

        # INFORMAZIONI SUI PEDONI
        lista_pedoni = self.pedestrian_avoid_manager(ego_waypoint_veicolo)
        stato_pedone, pedone, distanza_pedone, is_pedone_nella_stessa_corsia= lista_pedoni[0] if len(lista_pedoni) > 0 else (False, None, -1, None)
        
        # INFORMAZIONI SUI CICLISTI
        lista_ciclisti = self.biker_avoid_manager(ego_waypoint_veicolo)
        stato_ciclista, ciclista, distanza_ciclista, is_ciclista_nella_stessa_corsia = lista_ciclisti[0] if len(lista_ciclisti) > 0 else (False, None, -1, None)
        
        # INFORMAZIONI SUGLI OSTACOLI STATICI
        lista_ostacoli = self.static_obstacle_avoid_manager(waypoint=waypoint_attuale, lane='same', static_obstacles=attori_statici_filtrati)        
        stato_ostacolo, ostacolo, distanza_ostacolo = lista_ostacoli[0] if len(lista_ostacoli) > 0 else (False, None, -1)
        
        # INFORMAZIONI SUI VEICOLI
        lista_veicoli = self.collision_and_car_avoid_manager(ego_waypoint_veicolo)
        stato_veicolo, veicolo, distanza_veicolo = lista_veicoli[0] if len(lista_veicoli) > 0 else (False, None, -1)


        #  INFORMAZIONE VEICOLO LATERALE 
        lista_veicoli_laterali = self.side_obstacle_avoid_manager(ego_waypoint_veicolo)
        stato_veicolo_laterale, veicolo_laterale, distanza_veicolo_laterale = lista_veicoli_laterali[0] if len(lista_veicoli_laterali) > 0 else (False, None, -1)

        # INFORMAZIONI SU OSTACOLI NELLA CORSIA DI SINISTRA
        lista_ostacoli_laterali_altra_linea = self.static_obstacle_avoid_manager(ego_waypoint_veicolo, lane="left",static_obstacles=attori_statici_filtrati)
        stato_ostacolo_laterale_altra_linea, ostacolo_laterale_altra_linea, distanza_ostacolo_laterale_altra_linea = \
                lista_ostacoli_laterali_altra_linea[0] if len(lista_ostacoli_laterali_altra_linea) > 0 else (False, None, -1)

        #  INFORMAZIONI SU INCROCI
        self._in_incrocio = ego_waypoint_veicolo.is_junction
        if self._waypoint_entrante.is_junction and self._lista_waypoint_incroci is None:
            self._lista_waypoint_incroci = self._local_planner.get_junction_waypoint()


        # Comportamento ai semafori rossi e agli stop
        if self.traffic_light_manager():
            return self.emergency_stop()
        


        if influenzato_dal_segnale_di_stop:
            if distanza_segnale_stop < self._behavior.braking_distance:
                print("STOP STATE - EMERGENCY STOP")
                return self.emergency_stop()
            else:
                print("STOP STATE - SLOWING DOWN")
                if stato_veicolo:
                    distanza = distanza_veicolo \
                    - max(
                        veicolo.bounding_box.extent.y, 
                        veicolo.bounding_box.extent.x) \
                            - max(
                                self._vehicle.bounding_box.extent.y, 
                                self._vehicle.bounding_box.extent.x)
            
                    
                    if distanza < self._behavior.braking_distance:
                        print("STOP STATE WITH VEHICLE - EMERGENCY STOP")
                        return self.emergency_stop()
                    else:
                        print("STOP STATE WITH VEHICLE STATE - CAR FOLLOWING")
                        controllo = self.car_following_manager(veicolo, distanza)
                else:
                    velocita_target = min([
                        self._behavior.max_speed,
                        self._limite_velocita - self._behavior.speed_lim_dist,
                        20])
                    self._local_planner.set_speed(velocita_target)
                    controllo = self._local_planner.run_step(debug=debug)

                return controllo
        
        # Behavior per pedoni, ciclisti e veicoli
        if stato_pedone:
            distanza = distanza_pedone - max(
                pedone.bounding_box.extent.y, pedone.bounding_box.extent.x) - max(
                self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)

            if distanza < self._behavior.braking_distance and (is_pedone_nella_stessa_corsia or round(get_speed(pedone), ndigits=1) > 0.0):
                print("WAKLER STATE - EMERGENCY STOP")
                return self.emergency_stop()
            else:
                print("WAKLER STATE - SLOWING DOWN")
                velocita_target = min([
                    self._behavior.max_speed,
                    self._limite_velocita - 20,
                    self._velocita_in_curva,
                    20]
                    )
                self._local_planner.set_speed(velocita_target)
                return self._local_planner.run_step(debug=debug)

        if stato_ciclista:
            direction = self.get_actor_moving_direction(ciclista.get_transform(), self._vehicle.get_transform())
            
            if direction == "forward": 
                # se il ciclista si sta muovendo in avanti, basta cambiare l'offset del veicolo
                print("BIKER STATE - BIKER ON THE RIGHT")
                print(self._local_planner._vehicle_controller._lat_controller.get_offset())
                self._local_planner._vehicle_controller._lat_controller._offset= -0.9
                return self._local_planner.run_step(debug=debug)
            else: 
                # altrimenti il ciclista sta attraversando la strada, stesso comportamento del pedone
                distanza = distanza_ciclista \
                    - max(
                        ciclista.bounding_box.extent.y,
                        ciclista.bounding_box.extent.x) \
                            - max(
                                self._vehicle.bounding_box.extent.y,
                                self._vehicle.bounding_box.extent.x)
                
                if distanza < self._behavior.braking_distance and (is_ciclista_nella_stessa_corsia or round(get_speed(ciclista), ndigits=1) > 0.0):
                    print("BIKER STATE - EMERGENCY STOP")
                    return self.emergency_stop()
                else:
                    print("BIKER STATE - SLOWING DOWN")
                    velocita_target = min([
                    self._behavior.max_speed,
                    self._limite_velocita - 20,
                    self._velocita_in_curva,
                    20]
                    )
                    self._local_planner.set_speed(velocita_target)
                    return self._local_planner.run_step(debug=debug)
       

        # ----- VEHICLE BEHAVIOR -----
        if stato_veicolo:
            distanza = distanza_veicolo - max(veicolo.bounding_box.extent.y, veicolo.bounding_box.extent.x) - max(self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)
            
            # freno di emergenza se l'auto è molto vicina
            if distanza < self._behavior.braking_distance:
                print("VEHICLE STATE - EMERGENCY STOP")
                self.front_ultimo_veicolo=veicolo.id
                return self.emergency_stop()
            else:
                print("VEHICLE STATE - CAR FOLLOWING")
                controllo = self.car_following_manager(veicolo, distanza)        

        elif stato_veicolo_laterale:
            print("SIDE VEHICLE STATE - OVERTAKE")
            if len(lista_veicoli_laterali) == 1:
                vl = lista_veicoli_laterali[0]
                side_vehicle_length = max(vl[1].bounding_box.extent.x, vl[1].bounding_box.extent.y) * 2
            else:
                side_vehicle_length = lista_veicoli_laterali[-1][2] - lista_veicoli_laterali[0][2]

            controllo = self.sorpassa(ego_waypoint_veicolo, veicolo_laterale, side_vehicle_length, distanza_veicolo_laterale)

        # 3: Intersection behavior
        elif self._waypoint_entrante.is_junction or self._in_incrocio:

            lista_veicoli_in_incrocio = self._world.get_actors().filter("*vehicle*")
            def dist(v): return v.get_location().distance(ego_waypoint_veicolo.transform.location)

            # ottieni la lista dei veicoli che rispettano una distanza limite e non sono biciclette
            lista_veicoli_in_incrocio = [v for v in lista_veicoli_in_incrocio if dist(v) < 45 and v.id != self._vehicle.id and not v.type_id in self._bike_type_list]

            is_veicolo_in_incrocio, velocita_veicolo = self._vehicle_obstacle_in_junction_detected(lista_veicoli_in_incrocio, max(
                        self._behavior.min_proximity_threshold, self._limite_velocita / 3))

            if self._direzione_entrante == RoadOption.RIGHT:
                self._local_planner._vehicle_controller._lat_controller.set_offset(-0.2)
            else:
                self._local_planner._vehicle_controller._lat_controller.set_offset(0.0)

            indice = 0
            lunghezza_totale = 0
            waypoint_attuale = self._local_planner.get_current_waypoint()

            if self._lista_waypoint_incroci is not None:
                try:
                    indice = self._lista_waypoint_incroci.index(waypoint_attuale.id) + 1
                    self._indice_ultimo_incrocio_waypoint = indice
                except:
                    indice = self._indice_ultimo_incrocio_waypoint
                
                lunghezza_totale = len(self._lista_waypoint_incroci)
            
            percentuale = (indice/lunghezza_totale) * 100
            is_incrocio_passato = percentuale > 70

            if not is_veicolo_in_incrocio or (self._in_incrocio and round(velocita_veicolo, ndigits=1) == 0.0) or is_incrocio_passato:
                print("INCOMING JUNCTION STATE - NORMAL BEHAVIOR")
                velocita_target = min([
                    self._behavior.max_speed,
                    self._limite_velocita - 15,
                    self._velocita_in_curva]
                    )
                self._local_planner.set_speed(velocita_target)
                controllo = self._local_planner.run_step(debug=debug)
            else:
                print("INCOMING JUNCTION STATE - EMERGENCY STOP")
                return self.emergency_stop()


        elif stato_ostacolo:
            print(f"INCOMING ACCIDENT STATE")

            if len(lista_ostacoli) == 1:
                os = lista_ostacoli[0]
                lunghezza_ostacolo = max(os[1].bounding_box.extent.x, os[1].bounding_box.extent.y) * 2
            else:
                lunghezza_ostacolo = lista_ostacoli[-1][2] - lista_ostacoli[0][2]

            controllo = self.sorpassa(ego_waypoint_veicolo, ostacolo, lunghezza_ostacolo, distanza_ostacolo)

        elif stato_ostacolo_laterale_altra_linea:
            print("SIDE OBSTACLE STATE - MOVING TO THE RIGHT")
            velocita_target = min([
                self._behavior.max_speed,
                self._limite_velocita - self._behavior.speed_lim_dist])

            self._local_planner.set_speed(velocita_target)
            self._local_planner._vehicle_controller._lat_controller._offset=0.70
            controllo = self._local_planner.run_step(debug=debug)
    
        else:
            print("ULTIMO STEP!!")
            self._lista_waypoint_incroci = None
            self._indice_ultimo_incrocio_waypoint = 0
            self.move_to_center()
            velocita_target = min([
                self._behavior.max_speed,
                self._limite_velocita - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(velocita_target)
            if self._direzione_entrante == RoadOption.RIGHT:
                self._local_planner._vehicle_controller._lat_controller.set_offset(-0.2)
            else:
                self.move_to_center()
            controllo = self._local_planner.run_step(debug=debug)

        return controllo
    
    def move_to_center(self):
        self._local_planner._vehicle_controller._lat_controller.set_offset(0.0)

    
    def stop_sign_manager(self):
        """
        This method is in charge of behaviors for stop sign.

        return: presence of stop sign and distance from it
        """
        lista_attori = self._world.get_actors()
        lista_segnali_stop = lista_attori.filter("*stop*")
        if self._ignora_stop is not None:
            lista_segnali_stop = [stop_sign for stop_sign in lista_segnali_stop if stop_sign.id != self._ignora_stop.id]
        
        affected, segnale_stop, distanza = self._affected_by_stop_sign(lista_segnali_stop)

        if round(self._velocita, ndigits=1) == 0.0 and affected:
            if self._stop_counter > 30:
                self._ignora_stop = segnale_stop
                self._stop_counter = 0
            else:
                self._stop_counter += 1

        return affected, distanza
    
    
    def _update_information(self):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.
        """
        self._velocita = get_speed(self._vehicle)
        self._limite_velocita = self._vehicle.get_speed_limit()
        self._local_planner.set_speed(self._limite_velocita)
        self._direzione = self._local_planner.target_road_option
        if self._direzione is None:
            self._direzione = RoadOption.LANEFOLLOW

        self.guarda_passi_avanti = int((self._limite_velocita) / 10)

        self._waypoint_entrante, self._direzione_entrante = self._local_planner.get_incoming_waypoint_and_direction(
            steps=self.guarda_passi_avanti)
        if self._direzione_entrante is None:
            self._direzione_entrante = RoadOption.LANEFOLLOW

    def traffic_light_manager(self):
        """
        This method is in charge of behaviors for red lights.
        """
        lista_attori = self._world.get_actors()
        lista_semafori = lista_attori.filter("*traffic_light*")
        affected, _ = self._affected_by_traffic_light(lista_semafori)

        return affected

    def _tailgating(self, waypoint, vehicle_list):
        """
        This method is in charge of tailgating behaviors.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """

        svolta_sinistra = waypoint.left_lane_marking.lane_change
        svolta_destra = waypoint.right_lane_marking.lane_change

        waypoint_sinistra = waypoint.get_left_lane()
        waypoint_destra = waypoint.get_right_lane()

        stato_veicolo_dietro, behind_vehicle, _ = self._vehicle_obstacle_detected(vehicle_list, max(
            self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=180, low_angle_th=160)
        if stato_veicolo_dietro and self._velocita < get_speed(behind_vehicle):
            if (svolta_destra == carla.LaneChange.Right or svolta_destra ==
                    carla.LaneChange.Both) and waypoint.lane_id * waypoint_destra.lane_id > 0 and waypoint_destra.lane_type == carla.LaneType.Driving:
                nuovo_stato_del_veicolo, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(
                    self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=180, lane_offset=1)
                if not nuovo_stato_del_veicolo:
                    print("Tailgating, moving to the right!")
                    waypoint_finale = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    self.set_destination(waypoint_finale.transform.location,
                                         waypoint_destra.transform.location)
            elif svolta_sinistra == carla.LaneChange.Left and waypoint.lane_id * waypoint_sinistra.lane_id > 0 and waypoint_sinistra.lane_type == carla.LaneType.Driving:
                nuovo_stato_del_veicolo, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(
                    self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=180, lane_offset=-1)
                if not nuovo_stato_del_veicolo:
                    print("Tailgating, moving to the left!")
                    waypoint_finale = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    self.set_destination(waypoint_finale.transform.location,
                                         waypoint_sinistra.transform.location)

    def collision_and_car_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        and managing possible tailgating chances.

            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a vehicle nearby, False if not
            :return vehicle: nearby vehicle
            :return distance: distance to nearby vehicle
        """
        lista_veicoli = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        # ottieni la lista dei veicoli che rispettano una distanza limite e non sono biciclette
        lista_veicoli = [v for v in lista_veicoli if dist(v) < 45 and v.id != self._vehicle.id and not v.type_id in self._bike_type_list]

        if self._direzione == RoadOption.CHANGELANELEFT:
            return self._vehicle_obstacle_detected(
                lista_veicoli, max(
                    self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=180, lane_offset=-1)
        elif self._direzione == RoadOption.CHANGELANERIGHT:
            return self._vehicle_obstacle_detected(
                lista_veicoli, max(
                    self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=180, lane_offset=1)
        else:
            return self._vehicle_obstacle_detected(
                lista_veicoli, max(
                    self._behavior.min_proximity_threshold, self._limite_velocita / 3), up_angle_th=30)


    def pedestrian_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        with any pedestrian.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a walker nearby, False if not
            :return vehicle: nearby walker
            :return distance: distance to nearby walker
        """


        lista_pedoni = self._world.get_actors().filter("*walker.pedestrian*")

        def dist(w):
            return w.get_location().distance(waypoint.transform.location)

        lista_pedoni = [w for w in lista_pedoni if dist(w) < 20]

        if self._direzione == RoadOption.CHANGELANELEFT:
            lista_pedoni_temporanea = self._walker_obstacle_detected(lista_pedoni, max(
                self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=90, lane_offset=-1)
        elif self._direzione == RoadOption.CHANGELANERIGHT:
            lista_pedoni_temporanea= self._walker_obstacle_detected(lista_pedoni, max(
                self._behavior.min_proximity_threshold, self._limite_velocita / 2), up_angle_th=90, lane_offset=1)
        else:
            lista_pedoni_temporanea = self._walker_obstacle_detected(lista_pedoni, max(
                self._behavior.min_proximity_threshold, self._limite_velocita / 3), up_angle_th=60)

        return lista_pedoni_temporanea
    

    def static_obstacle_avoid_manager(self, waypoint, lane='same', static_obstacles=None):
        """
        This module is in charge of warning in case of a collision with any static obstacle.

        :param waypoint: current waypoint of the agent
        :param lane: location of obstacle: in same line or on the left
        :param static_obstacles: Optional[List[carla.Actor]] - obstacles passed externally
        :return: sorted list of detected static obstacles (with metadata)
        """

        # Se non sono forniti, li prendo dal mondo (retrocompatibilità)
        if static_obstacles is None:
            lista_ostacoli = self._world.get_actors().filter("*static.prop*")

            def dist(v):
                return v.get_location().distance(waypoint.transform.location)

            lista_ostacoli_temporanea = [o for o in lista_ostacoli if dist(o) < 30 and "dirtdebris" not in o.type_id]
        else:
            lista_ostacoli_temporanea = static_obstacles

        ego_transform = self._vehicle.get_transform()

        # Ottieni la trasformazione della parte anteriore del veicolo ego
        ego_vettore_forward = ego_transform.get_forward_vector()
        ego_extent = self._vehicle.bounding_box.extent.x
        ego_front_transform = ego_transform
        ego_front_transform.location += carla.Location(
            x=ego_extent * ego_vettore_forward.x,
            y=ego_extent * ego_vettore_forward.y,
        )

        lista_ostacoli = []

        for ostacoli in lista_ostacoli_temporanea:
            transform_ostacoli = ostacoli.get_transform()
            waypoint_ostacoli = self._map.get_waypoint(transform_ostacoli.location, lane_type=carla.LaneType.Any)
            

            if lane == "left":
                if waypoint_ostacoli.road_id == waypoint.road_id and waypoint_ostacoli.lane_id != waypoint.lane_id and is_within_distance(
                        transform_ostacoli, ego_front_transform, max_distance=50, angle_interval=[0, 135]):
                    lista_ostacoli.append(
                        (True, ostacoli, compute_distance(transform_ostacoli.location, ego_transform.location)))
                    
            else:
                if waypoint_ostacoli.road_id == waypoint.road_id and waypoint_ostacoli.lane_id == waypoint.lane_id and is_within_distance(
                        transform_ostacoli, ego_front_transform, max_distance=50, angle_interval=[0, 30]):
                    lista_ostacoli.append(
                        (True, ostacoli, compute_distance(transform_ostacoli.location, ego_transform.location)))

        lista_ostacoli_ordinata = sorted(lista_ostacoli, key=lambda x: x[2])

        return lista_ostacoli_ordinata

    def car_following_manager(self, vehicle, distance, debug=False):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.

            :param vehicle: car to follow
            :param distance: distance from vehicle
            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """

        velocita_veicolo = get_speed(vehicle) 
        delta_v = max(1, (self._velocita - velocita_veicolo) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / np.nextafter(0., 1.)

        # Sotto la distanza di tempo di sicurezza, rallenta.
        if self._behavior.safety_time > ttc > 0.0:
            velocita_target = min([
                positive(velocita_veicolo - self._behavior.speed_decrease),
                self._behavior.max_speed,
                self._limite_velocita - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(velocita_target)
            controllo = self._local_planner.run_step(debug=debug)

        # Area di distanza di sicurezza effettiva, cerca di seguire la velocità del veicolo davanti.
        elif 2 * self._behavior.safety_time > ttc >= self._behavior.safety_time:
            velocita_target = min([
                max(self._velocita_minima, velocita_veicolo),
                self._behavior.max_speed,
                self._limite_velocita - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(velocita_target)
            controllo = self._local_planner.run_step(debug=debug)

        #behavior Normale 
        else:
            velocita_target = min([
                self._behavior.max_speed,
                self._limite_velocita - self._behavior.speed_lim_dist])
            self._local_planner.set_speed(velocita_target)
            controllo = self._local_planner.run_step(debug=debug)

        return controllo

    def emergency_stop(self):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control


    def _genera_percorso_per_sorpasso(self, distance_for_overtaking, direction='left', two_way=False):

        """
        Method to create a path for overtaking.

        :param distance_for_overtaking: The distance the vehicle needs to travel to complete the overtaking maneuver.
        :param direction: The direction of the overtaking maneuver ('left' for overtaking to the left, 'right' for overtaking to the right).
        :param two_way: A boolean indicating if the road is a two-way road, which affects the maneuver strategy.

        :return: None. The method modifies the global plan of the vehicle's path by setting a new plan
        """

        waypoint = self._map.get_waypoint(self._vehicle.get_location())
        step_distanza = self._risoluzione_campionamento
        vecchio_planner = self._local_planner._waypoints_queue

        planner = []
        planner.append((waypoint, RoadOption.LANEFOLLOW))

        prossimo_waypoint = waypoint.next(step_distanza)[0]

        if direction == 'left':
            waypoint_laterale = prossimo_waypoint.get_left_lane()
            planner.append((prossimo_waypoint, RoadOption.CHANGELANELEFT))
            planner.append((waypoint_laterale, RoadOption.LANEFOLLOW))
        else:
            waypoint_laterale = prossimo_waypoint.get_lane_right()
            planner.append((prossimo_waypoint, RoadOption.CHANGELANERIGHT))
            planner.append((waypoint_laterale, RoadOption.LANEFOLLOW))

        distanza = 0
        while distanza < distance_for_overtaking:
            if two_way:
                next_wps = planner[-1][0].previous(step_distanza)
            else:
                next_wps = planner[-1][0].next(step_distanza)
            prossimo_waypoint = next_wps[0]

            distanza += prossimo_waypoint.transform.location.distance(planner[-1][0].transform.location)
            planner.append((prossimo_waypoint, RoadOption.LANEFOLLOW))

        if two_way:
            prossimo_waypoint = planner[-1][0].previous(10)[0]
            waypoint_laterale = prossimo_waypoint.get_left_lane()
            planner.append((waypoint_laterale, RoadOption.CHANGELANERIGHT))
        else:
            prossimo_waypoint = planner[-1][0].next(10)[0]
            waypoint_laterale = prossimo_waypoint.get_left_lane()
            planner.append((waypoint_laterale, RoadOption.CHANGELANERIGHT))

        waypoint_vecchio_planner = list(map(lambda x: x[0], vecchio_planner))
        for i in range(self._global_planner._find_closest_in_list(planner[-1][0], waypoint_vecchio_planner), len(waypoint_vecchio_planner)):
            planner.append(self._local_planner._waypoints_queue[i])

        self.set_global_plan(planner)

    def sorpassa(self, waypoint, target, target_length, target_distance, target_speed=0, debug=False, security_distance = 3):
        
        """
        Method to perform an overtaking maneuver.

        :param waypoint: The current waypoint of the ego vehicle.
        :param target: The vehicle to be overtaken.
        :param target_length: The length of the target vehicle.
        :param target_distance: The distance to the target vehicle.
        :param target_speed: The speed of the target vehicle (default is 0).
        :param debug: A boolean indicating whether to enable debug mode (default is False).
        :param security_distance: The additional safety distance to consider (default is 3 meters).

        :return: The control command for the vehicle to execute the overtaking maneuver.
        """
        
        distanza_sorpasso = target_distance + target_length
        velocità = min([self._behavior.max_speed, self._limite_velocita - self._behavior.speed_lim_dist]) / 3.6
        tempo_sorpasso = distanza_sorpasso / (velocità - target_speed)

        stato_di_pericolo, veicolo_pericoloso, distanza_dal_pericolo = self._controllo_veicoli_in_corsia_di_sorpasso(waypoint, 100)

        if not stato_di_pericolo: 
            # nessun pericolo nel sorpasso
            if target_distance < self._behavior.braking_distance + security_distance :
                self._genera_percorso_per_sorpasso(distanza_sorpasso - 1.5, 'left', True)
                self._local_planner.set_speed(velocità * 3.6)
                controllo = self._local_planner.run_step(debug=debug)
            else:
                controllo = self._local_planner.run_step(debug=debug)
        else: 
            # veicolo nella corsia di sorpasso
            pericolo_distanza_veicolo = get_speed(veicolo_pericoloso) / 3.6
            s1 = distanza_dal_pericolo - pericolo_distanza_veicolo * (tempo_sorpasso + 3)

            if s1 > distanza_sorpasso + 5:
                if target_distance < self._behavior.braking_distance + security_distance:
                    self._genera_percorso_per_sorpasso(distanza_sorpasso - 1.5, 'left', True)
                    self._local_planner.set_speed(velocità * 3.6)
                    controllo = self._local_planner.run_step(debug=debug)
                else:
                    controllo = self._local_planner.run_step(debug=debug)
            else:
                if target_distance < self._behavior.braking_distance + security_distance:
                    return self.emergency_stop()

                controllo = self.car_following_manager(target, target_distance)

        return controllo
    
    def _controllo_veicoli_in_corsia_di_sorpasso(self, waypoint, distanza):
        """
        Method to check for the presence of a vehicle on the overtaking lane.

        :param waypoint: The current waypoint of the ego vehicle.
        :param distance: The distance within which to check for other vehicles on the overtaking lane.

        :return: A tuple containing:
            - danger_state (bool): Indicates if there is a vehicle on the overtaking lane within the specified distance.
            - def_danger_vehicle (carla.Actor or None): The vehicle posing the danger if any, otherwise False.
            - danger_distance (float): The distance to the nearest vehicle on the overtaking lane.
        """
        lista_veicoli_pericolosi= self._world.get_actors().filter("*vehicle*")

        def dist(v): return v.get_location().distance(waypoint.transform.location)
        
        waypoint_corsia_sinistra = waypoint.get_left_lane()
        id_corsia_sinistra = waypoint_corsia_sinistra.lane_id

        stato_di_pericolo = False
        def_veicolo_pericoloso = False
        distanza_dal_pericolo = distanza

        for veicolo_pericoloso in lista_veicoli_pericolosi:
            transform_pericolo = veicolo_pericoloso.get_transform()
            waypoint_pericolo = self._map.get_waypoint(transform_pericolo.location, lane_type=carla.LaneType.Any)

            if waypoint_pericolo.road_id == waypoint.road_id and waypoint_pericolo.lane_id == id_corsia_sinistra:
                distanza = dist(veicolo_pericoloso)
                if is_within_distance(transform_pericolo, waypoint.transform, distanza_dal_pericolo, [0, 90]):
                    stato_di_pericolo = True
                    def_veicolo_pericoloso = veicolo_pericoloso
                    distanza_dal_pericolo = distanza

        return (stato_di_pericolo, def_veicolo_pericoloso, distanza_dal_pericolo)