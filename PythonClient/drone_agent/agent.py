import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint

from multiprocessing import Pipe
import haversine
import math

class DroneAgent():
    """
    Agent for Drone
    """
    def __init__(self, leader=True, input_pipe=None, output_pipe=None, UE=True, droneID=''):
        """
        leader : Initialize agent's role
            True : leader
            False : follower
        input_pipe : Initialize input_pipe for multiprocessing
        output_pipe : Initialize output_pipe for multiprocessing
        UE : Initialize environment is Unreal Engine or ROS
            True : Unreal Engine
            False : ROS
        client : Initialize client if environment is Unreal Engine
        droneID : Initialize drone's id
        """
        self._leader = leader
        self._input_pipe = input_pipe
        self._output_pipe = output_pipe
        self._UE = UE
        if self._UE:
            self._client = airsim.MultirotorClient()
        self._droneID = droneID
    
    def set_role(self, leader=True):
        """
        Set agent's role
            True : leader
            False : follower
        """
        self._leader = leader

    def get_role(self):
        """
        Get agent's role
            True : leader
            False : follower
        """
        return self._leader

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
        position : (x, y, z) 3d position tuple
        velocity : moving velosity (m/s)
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
                    path = self._global_path_list[index]
                    velocity = self._global_velocity_list[index]
                    self._client.moveToPositionAsync(path[0], path[1], path[2], velocity, vehicle_name=self._droneID).join()
        else:
            if self._leader:
                pass
    
    def collision_avoidance(self, weight=1, gpses=[], boundary=1):
        """
        Calculate vector for the rule of collision avoidance
        weight : the weight for the rule of collision avoidance
        gpses : the gpses of other drones
        boundary : the boundary of the flocking group by self
        """
        pass

    def velocity_matching(self, weight=1, gpses=[], boundary=1):
        """
        Calculate vector for the rule of velocity matching
        weight : the weight for the rule of velocity matching
        gpses : the gpses of other drones
        boundary : the boundary of the flocking group by self
        """
        pass

    def flocking_center(self, weight=1, gpses=[], boundary=1):
        """
        Calculate vector for the rule of flocking center
        weight : the weight for the rule of flocking center
        gpses : the gpses of other drones
        boundary : the boundary of the flocking group by self
        """
        # get drone's location
        self._location = self._client.getMultirotorState
        pass

    def flocking_flight(self, weights, gpses=[], boundary=1):
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
                    ground_distance = haversine.Haversine((self._gps.longitude, self._gps.latitude), (gpses[i].longitude, gpses[i].latitude))
                    real_distance = math.sqrt(ground_distance**2 + (self._gps.altitude - gpses[i].altitude)**2)
                pass 
        else:
            pass

    def formation_flight(self):
        """
        Agent command drone to fly by formation
        """
        pass