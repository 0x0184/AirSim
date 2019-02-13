import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint

from multiprocessing import Pipe
import haversine
import math

class DroneAgent:
    """
    Agent for Drone
    """
    def __init__(self, leader=True, input_pipe=[], output_pipe=[], UE=True, droneID='', neighbor_distance=5, neighbor_angle=180):
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
        self._location = self._client.getMultirotorState().gps_location
        self._linear_velocity = self._client.getMultirotorState().kinematics_estimated.linear_velocity
        self._angular_velocity = self._client.getMultirotorState().kinematics_estimated.angular_velocity
        
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
                for index in range(len(self._global_path_list)):
                    # check use moveOnPath
                    path = self._global_path_list[index]
                    velocity = self._global_velocity_list[index]
                    self._client.moveToPositionAsync(path[0], path[1], path[2], velocity, vehicle_name=self._droneID).join()
        else:
            if self._leader:
                pass
    
    def collision_avoidance(self, weight=1, gpses=[]):
        """
        Calculate vector for the rule of collision avoidance
        weight: the weight for the rule of collision avoidance
        gpses: the gpses of other drones
        boundary: the boundary of the flocking group by self
        """
        pass

    def velocity_matching(self, weight=1, gpses=[]):
        """
        Calculate vector for the rule of velocity matching
        weight: the weight for the rule of velocity matching
        gpses: the gpses of other drones
        boundary: the boundary of the flocking group by self
        """
        pass

    def flocking_center(self, weight=1, gpses=[]):
        """
        Calculate vector for the rule of flocking center
        weight: the weight for the rule of flocking center
        gpses: the gpses of other drones
        boundary: the boundary of the flocking group by self
        """
        # get drone's location
        self._location = self._client.getMultirotorState
        pass

    def flocking_flight(self, weights, gpses=[]):
        """
        Agent command drone to fly by flocking
        """
        if self._UE:
            if not self._leader:
                # collect other drone's location
                self._gps = self._client.getMultirotorState().gps_location

                # check distance is less than boundary
                distance_check = []
                
                for i in range(len(gpses)):
                    ground_distance = haversine.Haversine3D((self._gps.latitude, self._gps.longitude, self._gps.altitude), (gpses[i].latitude, gpses[i].longitude, gpses[i].altitude))

                    if ground_distance < self._neighbor_distance:
                        distance_check.append(True)
                    else:
                        distance_check.append(False)
                

        else:
            pass

    def formation_flight(self):
        """
        Agent command drone to fly by formation
        """
        pass