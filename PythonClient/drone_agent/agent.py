import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint

from multiprocessing import Pipe
import math
import localmap
import vector

class DroneAgent:
    """
    Agent for Drone
    """
    def __init__(self, leader=True, input_pipe=[], output_pipe=[], UE=True, droneID='', neighbor_distance=5, neighbor_angle=180, local_map=localmap.LocalMap()):
        """
        leader: Initialize agent's role
            True: leader
            False: follower
        input_pipe: Initialize list of input_pipe for multiprocessing
            0 index must be ground control
        output_pipe: Initialize list of output_pipe for multiprocessing
            0 index must be ground control
        UE: Initialize environment is Unreal Engine or ROS
            True: Unreal Engine
            False: ROS
        client: Initialize client if environment is Unreal Engine
        droneID: Initialize drone's id
        distance: Initialize agent's flocking neighborhood boundary distance(m)
        angle: Initialize agent's flocking neighborhood boundary angle(0~180)
        """
        self._leader = leader
        self._input_pipe = input_pipe
        self._output_pipe = output_pipe
        self._UE = UE
        if self._UE:
            self._client = airsim.MultirotorClient()
        self._droneID = droneID
        self._neighbor_distance = neighbor_distance
        self._neighbor_angle = neighbor_angle
        self._linear_velocity = self._client.getMultirotorState().kinematics_estimated.linear_velocity
        self._angular_velocity = self._client.getMultirotorState().kinematics_estimated.angular_velocity
        self._local_map = local_map
        
    def set_role(self, leader=True):
        """
        Set agent's role
            True: leader
            False: follower
        """
        self._leader = leader

    def get_role(self):
        """
        Get agent's role
            True: leader
            False: follower
        """
        return self._leader

    def set_neighbor_distance(self, neighbor_distance):
        """
        Set agent's flocking neighborhood boundary distance(m)
        """
        self._neighbor_distance = neighbor_distance

    def get_neighbor_distance(self):
        """
        Get agent's flocking neighborhood boundary distance(m)
        """
        return self._neighbor_distance

    def set_neighbor_angle(self, neighbor_angle):
        """
        Set agent's flocking neighborhood boundary angle(0~180)
        """
        self._neighbor_angle = neighbor_angle

    def get_neighbor_angle(self, neighbor_angle):
        """
        Get agent's flocking neighborhood boundary angle(0~180)
        """
        return self._neighbor_angle

    def set_global_path(self, global_path_list, global_velocity_list):
        """
        Set leader agent's global path list
        """
        if self._leader:
            self._global_path_list = global_path_list
            self._global_velocity_list = global_velocity_list
            self._path_index = 0
        else:
            pass

    def get_global_path(self):
        """
        get leader agent's global path list and path index
        """
        if self._leader:
            return (self._global_path_list, self._global_velocity_list, self._path_index)
        else:
            return (None, None)

    def check_connection(self):
        """
        Agent check connection with Drone
        """
        if self._UE:
            self._client.confirmConnection()

    def takeoff(self):
        """
        Agent command drone to take off
        """
        if self._UE:
            self._client.enableApiControl(True, self._droneID)
            self._client.armDisarm(True, self._droneID)
            self._client.takeoffAsync(vehicle_name=self._droneID).join()

    def land(self):
        """
        Agent commend drone to land
        """
        if self._UE:
            self._client.landAsync(vehicle_name=self._droneID).join()
            self._client.armDisarm(False, self._droneID)
            self._client.enableApiControl(False, self._droneID)

    def moveToPosition(self, position, velocity):
        """
        Agent command drone to fly to specific position
        position: (x, y, z) 3d position tuple
        velocity: moving velosity (m/s)
        """
        if self._UE:
            self._client.moveToPositionAsync(position[0], position[1], position[2], velocity, vehicle_name=self._droneID).join()
            self._client.hoverAsync(vehicle_name=self._droneID).join()

    def path_fly(self):
        """
        Agent command drone to fly with global path
        """
        if self._UE:
            if self._leader:
                ### check use moveOnPath
                for index in range(len(self._global_path_list)):
                    path = self._global_path_list[index]
                    velocity = self._global_velocity_list[index]
                    self._client.moveToPositionAsync(path[0], path[1], path[2], velocity, vehicle_name=self._droneID).join()
        else:
            if self._leader:
                pass
    
    def collision_avoidance(self, weight=1, locations=[], visible=[]):
        """
        Calculate vector for the rule of collision avoidance
        weight: the weight for the rule of collision avoidance
        gpses: the gpses of other drones
        boundary: the boundary of the flocking group by self
        """
        steer = vector.Vector()
        vec_sum = vector.Vector()
        count = 0

        for i in range(len(locations)):
            ### check / (distance ** 2) other code just / distance
            if visible[i]:
                count += 1
                distance = localmap.distance3D(loc3d1=self._location, loc3d2=locations[i])
                vec_sum += self._location - locations[i] / (distance**2)

        vec_sum /= count
        steer = vec_sum.normalize() * weight

        return steer

    def velocity_matching(self, weight=1, velocities=[], visible=[]):
        """
        Calculate vector for the rule of velocity matching
        weight: the weight for the rule of velocity matching
        gpses: the gpses of other drones
        boundary: the boundary of the flocking group by self
        """
        steer = vector.Vector()
        vel_sum = vector.Vector()
        
        for i in range(len(velocities)):
            if visible[i]:
                count += 1
                vel_sum += velocities[i]

        vel_sum /= count
        steer = vel_sum.normalize() * weight

        return steer

    def flocking_center(self, weight=1, locations=[], visible=[]):
        """
        Calculate vector for the rule of flocking center
        weight: the weight for the rule of flocking center
        gpses: the gpses of other drones
        boundary: the boundary of the flocking group by self
        """
        steer = vector.Vector()
        center = vector.Vector()
        count = 0

        for i in range(len(locations)):
            if visible[i]:
                count += 1
                center += locations[i]

        center /= count
        steer = center.normalize() * weight
        
        return steer

    def flocking_flight(self, weights=[], locations=[], velocities=[], max_speed=1):
        """
        Agent command drone to fly by flocking
        locations = [(x1, y1, z1), (x2, y2, z2), ...]
        """
        if self._UE:
            if not self._leader:
                # collect other drone's location
                self._location = self._client.getMultirotorState().kinematics_estimated.position
                self._location.z_val *= -1

                # check distance is less than boundary
                visible = []
                
                for i in range(len(locations)):
                    distance_on_map = localmap.distance3Dv((self._location.x_val, self._location.y_val, self._location.z_val), (locations[i].x_val, locations[i].y_val, locations[i].z_val))
                    
                    ### check angle also
                    if distance_on_map < self._neighbor_distance:
                        visible.append(True)
                    else:
                        visible.append(False)

                col_avo = self.collision_avoidance(weight=weights[0], locations=locations, visible=visible)
                vel_mat = self.velocity_matching(weight=weights[1], velocities=velocities, visible=visible)
                flo_cet = self.flocking_center(weight=weights[2], locations=locations, visible=visible)
                steer = (col_avo + vel_mat + flo_cet) * max_speed
        else:
            pass

    def formation_flight(self):
        """
        Agent command drone to fly by formation
        """
        pass