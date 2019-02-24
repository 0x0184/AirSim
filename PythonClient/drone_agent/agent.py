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
import haversine

class DroneAgent:
    """
    Agent for Drone
    """
    def __init__(self, leader=True, UE=True, conn=Pipe()[1], droneID='', error=[0, 0, 0], seperation_boundary = 2, neighbor_distance=25, neighbor_angle=180, local_map=localmap.LocalMap(coords=[], UE=True), follower_speed_multiplier=1.5):
        """
        leader: Initialize agent's role
            True: leader
            False: follower
        UE: Initialize environment is Unreal Engine or ROS
            True: Unreal Engine
            False: ROS
        conn: child_conn of Pipe() to connection with GCS
        droneID: Initialize drone's id
        distance: Initialize agent's flocking neighborhood boundary distance(m)
        angle: Initialize agent's flocking neighborhood boundary angle(0~180)
        local_map: map of specific gps area
        """
        self._leader = leader
        self._UE = UE
        if self._UE:
            self._client = airsim.MultirotorClient()
            self._conn = conn
            self._duration = 2
        self._droneID = droneID
        self._error = error
        self._seperation_boundary = seperation_boundary
        self._neighbor_distance = neighbor_distance
        self._neighbor_angle = neighbor_angle
        self._local_map = local_map
        self._follower_speed_multiplier = follower_speed_multiplier
        filename = self._droneID + '_flight.log'
        self._log = open('C:\\Users\\DsLiner\\AirSim\\PythonClient\\drone_agent\\log\\'+filename, 'w')
        self._velocity = vector.Vector()
        
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
        Set agent's global path list
        """
        if self._UE:
            self._global_path_list = global_path_list
            self._global_velocity_list = global_velocity_list
            self._path_index = 0
        else:
            pass

    def get_global_path(self):
        """
        get agent's global path list and path index
        """
        if self._UE:
            return (self._global_path_list, self._global_velocity_list, self._path_index)
        else:
            return (None, None)

    def check_path(self, path_boundary=1):
        """
        check drone is on which path
        """
        if self._UE:
            if self._leader:
                if localmap.distance3Dv(loc3d1=self._location, loc3d2=self._global_path_list[self._path_index]) < path_boundary:
                    self._path_index += 1

    def check_end(self, path_boundary=1):
        """
        check mission is ended
        """
        if localmap.distance3Dv(loc3d1=self._location, loc3d2=self._global_path_list[-1]) < path_boundary:
            return True
        else:
            return False
        
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
            self._client.enableApiControl(True, vehicle_name=self._droneID)
            self._client.armDisarm(True, vehicle_name=self._droneID)
            self._client.takeoffAsync(vehicle_name=self._droneID).join()

    def land(self, land_speed=1):
        """
        Agent commend drone to land
        """
        if self._UE:
            # airsim's landAsync command is unstable
            self._client.moveToPositionAsync(self._location.x_val, self._location.y_val, 0, land_speed, vehicle_name=self._droneID).join()
            self._client.landAsync(vehicle_name=self._droneID).join()
            self._client.armDisarm(False, vehicle_name=self._droneID)
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
    
    def collision_avoidance(self, weight=1, locations=[], visible=[]):
        """
        Calculate vector for the rule of collision avoidance
        weight: the weight for the rule of collision avoidance
        gpses: the gpses of other drones
        """
        steer = vector.Vector()
        vec_sum = vector.Vector()
        count = 0

        for i in range(len(locations)):
            if visible[i]:
                distance = localmap.distance3Dv(loc3d1=self._location, loc3d2=locations[i])
                if distance < self._seperation_boundary:
                    count += 1
                    difference = vector.Vector() + self._location - locations[i]
                    vec_sum += difference.normalize() / distance

        if count is 0:
            return steer
        else:
            steer = vec_sum / count

        return steer.normalize() * weight

    def velocity_matching(self, weight=1, velocities=[], visible=[], path_boundary=1, max_speed=5):
        """
        Calculate vector for the rule of velocity matching
        if leader, then agent fly with global path
        weight: the weight for the rule of velocity matching
        gpses: the gpses of other drones
        """
        if self._leader:
            ### check use moveOnPath
            if self.check_end(path_boundary=path_boundary):
                self._client.hoverAsync().join()
                return
            self.check_path(path_boundary=path_boundary)

            steer = vector.Vector()
            steer = self._global_path_list[self._path_index] - self._location
        else:
            steer = vector.Vector()
            vel_sum = vector.Vector()
            count = 0
            
            for i in range(len(velocities)):
                if visible[i]:
                    count += 1
                    vel_sum += (vector.Vector() + velocities[i])

            if count is 0:
                return steer
            else:
                vel_sum /= count
                steer = vel_sum.normalize() - (vector.Vector() + self._velocity).normalize()

        return steer.make_steer(max_speed=self._global_velocity_list[self._path_index]) * weight

    def flocking_center(self, weight=1, locations=[], visible=[]):
        """
        Calculate vector for the rule of flocking center
        weight: the weight for the rule of flocking center
        gpses: the gpses of other drones
        """
        steer = vector.Vector()
        center = vector.Vector()
        count = 0

        for i in range(len(locations)):
            if visible[i]:
                count += 1
                center += (vector.Vector() + locations[i])

        if count is 0:
            return steer
        else:
            center /= count
            steer = vector.Vector() + center - self._location
        
        return steer.normalize() * weight

    def flocking_flight(self, weights=[1, 1, 1]):
        """
        Agent command drone to fly by flocking
        """
        if self._UE:
            # collect drone's location
            self._location = self._client.getMultirotorState(vehicle_name=self._droneID).kinematics_estimated.position
            self._location.x_val += self._error[0]
            self._location.y_val += self._error[1]
            self._location.z_val += self._error[2]
            # self._angular_velocity = self._client.getMultirotorState().kinematics_estimated.angular_velocity

            self._conn.send({'location': self._location, 'velocity': self._velocity})

            data = self._conn.recv()
            locations = []
            velocities = []
            if data[0] == 'broking':
                for data_dict in data[1]:
                    locations.append(data_dict['location'])
                    velocities.append(data_dict['velocity'])

            # check distance is less than boundary
            visible = []
            
            for location in locations:
                distance_on_map = localmap.distance3Dv(self._location, location)
                
                ### check angle also
                if distance_on_map < self._neighbor_distance and distance_on_map > 0:
                    visible.append(True)
                else:
                    visible.append(False)
                    
            col_avo = self.collision_avoidance(weight=weights[0], locations=locations, visible=visible)
            vel_mat = self.velocity_matching(weight=weights[1], velocities=velocities, visible=visible, path_boundary= 1, max_speed=self._global_velocity_list[self._path_index])
            flo_cet = self.flocking_center(weight=weights[2], locations=locations, visible=visible)
            steer = (col_avo + vel_mat + flo_cet)
            steer.make_steer(max_speed=self._global_velocity_list[self._path_index])
            self._client.moveByVelocityAsync(steer.x_val, steer.y_val, steer.z_val, self._duration, vehicle_name=self._droneID)
            self._velocity = steer

            log_location = vector.Vector() + self._location
            log_velocity = vector.Vector() + self._velocity
            self._log.write('col_avo: '+col_avo.toString()+'\n')
            self._log.write('vel_mat: '+vel_mat.toString()+'\n')
            self._log.write('flo_cet: '+flo_cet.toString()+'\n')
            self._log.write('log_location: '+log_location.toString()+'\n')
            self._log.write('log_velocity: '+log_velocity.toString()+'\n\n')
        else:
            pass

    def formation_flight(self):
        """
        Agent command drone to fly by formation
        """
        pass

    def collect_data(self):
        """
        Collect Agent's data
        """
        self._location = self._client.getMultirotorState().kinematics_estimated.position
        self._location.x_val += self._error[0]
        self._location.y_val += self._error[1]
        self._location.z_val += self._error[2]
        self._conn.send([self._droneID, self._location])

def run_agent(conn, leader=True, UE=True, droneID='', error=[0, 0, 0], seperation_boundary=2, neighbor_distance=5, neighbor_angle=180, local_map=localmap.LocalMap(coords=[], UE=True)):
    """
    run drone agent with gcs command for Unreal Engine with AirSim
    conn: child_conn of Pipe() to connection with GCS
    leader: Initialize agent's role
        True: leader
        False: follower
    UE: Initialize environment is Unreal Engine or ROS
        True: Unreal Engine
        False: ROS
    client: Initialize client if environment is Unreal Engine
    droneID: Initialize drone's id
    distance: Initialize agent's flocking neighborhood boundary distance(m)
    angle: Initialize agent's flocking neighborhood boundary angle(0~180)
    local_map: map of specific gps area
    """
    droneAgent = DroneAgent(leader=leader, UE=UE, conn=conn, droneID=droneID, seperation_boundary=seperation_boundary, neighbor_distance=neighbor_distance, neighbor_angle=neighbor_angle, local_map=local_map, error=error)
    while True:
        if UE:
            command = conn.recv()
            if command[0] == 'set_global_path':
                droneAgent.set_global_path(global_path_list=command[1], global_velocity_list=command[2])
            elif command[0] == 'collect_data':
                droneAgent.collect_data()
            elif command[0] == 'takeoff':
                droneAgent.takeoff()
            elif command[0] == 'flocking_flight':
                droneAgent.flocking_flight(weights=command[1])
            elif command[0] == 'land':
                droneAgent.land()
            elif command[0] == 'end':
                break

