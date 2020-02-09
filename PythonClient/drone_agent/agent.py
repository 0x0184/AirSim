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

import socket
import random
import time

class DroneAgent:
    """
    Agent for Drone
    """
    def __init__(self, leader=True, SITL=True, conn=Pipe()[1], droneID='', start_location=[0, 0, 0], seperation_boundary = 25.0, neighbor_boundary=50, neighbor_angle=180, local_map=localmap.LocalMap(coords=[], SITL=True), maxspeed_weight=2):
        """
        leader: Initialize agent's role
            True: leader
            False: follower
        SITL: Initialize environment is Unreal Engine or ROS
            True: Unreal Engine
            False: ROS
        conn: child_conn of Pipe() to connection with GCS
        droneID: Initialize drone's id
        distance: Initialize agent's flocking neighborhood boundary distance(m)
        angle: Initialize agent's flocking neighborhood boundary angle(0~180)
        local_map: map of specific gps area
        """
        self._leader = leader
        self._SITL = SITL
        if self._SITL:
            self._client = airsim.MultirotorClient()
            self._conn = conn
            self._duration = 2
        else:
            self._host = '127.0.0.1'
            self._port = 10000
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
        self._droneID = droneID
        self._start_location = start_location
        self._seperation_boundary = seperation_boundary
        self._neighbor_boundary = neighbor_boundary
        self._neighbor_angle = neighbor_angle
        self._local_map = local_map
        filename = self._droneID + '_flight.log'
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self._log = open(dir_path+'\\log\\'+filename, 'w')
        self._velocity = vector.Vector()
        self._maxspeed = 1
        self._maxspeed_weight = maxspeed_weight
        self._maxforce = 1.5
        self._avoidance_radius = 1.0    # m
        self._minimum_avodance_boundary = 2.0   # m
        self._object_boundary = 1.0 # m
        self._maximum_avoidance_coefficient = 10.0  # m
        # self._gps_error_xy = 0  # m
        # self._gps_error_z = 0   # m
        # self._min_lte_delay = 0 # ms
        # self._max_lte_delay = 0 # ms
        self._gps_error_xy = 0.7    # m
        self._gps_error_z = 1.4     # m
        self._min_lte_delay = 70    # ms
        self._max_lte_delay = 113   # ms

    def gps_error(self, vector):
        error_xy = random.random() * self._gps_error_xy
        error_degree = random.random() * math.pi * 2

        error_x = error_xy * math.cos(error_degree)
        error_y = error_xy * math.sin(error_degree)
        error_z = random.random() * self._gps_error_z * 2 - self._gps_error_z

        vector.x_val += error_x
        vector.y_val += error_y
        vector.z_val += error_z

        return vector

    def lte_delay(self):
        interval = self._max_lte_delay - self._min_lte_delay

        delay_time = random.random() * interval + self._min_lte_delay   # ms

        time.sleep(delay_time / 1000)
        
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

    def set_neighbor_boundary(self, neighbor_boundary):
        """
        Set agent's flocking neighborhood boundary distance(m)
        """
        self._neighbor_boundary = neighbor_boundary

    def get_neighbor_boundary(self):
        """
        Get agent's flocking neighborhood boundary distance(m)
        """
        return self._neighbor_boundary

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
        if self._SITL:
            self._global_path_list = global_path_list
            self._global_velocity_list = global_velocity_list
            self._path_index = 0
        else:
            pass

    def get_global_path(self):
        """
        get agent's global path list and path index
        """
        if self._SITL:
            return (self._global_path_list, self._global_velocity_list, self._path_index)
        else:
            return (None, None)

    def check_path2D(self, check_boundary=2):
        """
        check drone is on which path
        """
        if self._SITL:
            if self._leader:
                if localmap.distance2Dv(loc2d1=self._location, loc2d2=self._global_path_list[self._path_index]) < check_boundary:
                    self._path_index += 1

    def check_path3D(self, check_boundary=2):
        """
        check drone is on which path
        """
        if self._SITL:
            if self._leader:
                if localmap.distance3Dv(loc3d1=self._location, loc3d2=self._global_path_list[self._path_index]) < check_boundary:
                    self._path_index += 1

    def check_end2D(self, check_boundary=2):
        """
        check mission is ended
        """
        if localmap.distance2Dv(loc2d1=self._location, loc2d2=self._global_path_list[-1]) < check_boundary and self._path_index >= len(self._global_path_list):
            return True
        else:
            return False

    def check_end3D(self, check_boundary=2):
        """
        check mission is ended
        """
        if localmap.distance3Dv(loc3d1=self._location, loc3d2=self._global_path_list[-1]) < check_boundary and self._path_index >= len(self._global_path_list):
            return True
        else:
            return False
        
    def check_connection(self):
        """
        Agent check connection with Drone
        """
        if self._SITL:
            self._client.confirmConnection()

    def takeoff(self):
        """
        Agent command drone to take off
        """
        if self._SITL:
            self._client.enableApiControl(True, vehicle_name=self._droneID)
            self._client.armDisarm(True, vehicle_name=self._droneID)
            self._client.takeoffAsync(vehicle_name=self._droneID).join()

    def land(self, land_speed=1):
        """
        Agent commend drone to land
        """
        if self._SITL:
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
        if self._SITL:
            self._client.moveToPositionAsync(position[0], position[1], position[2], velocity, vehicle_name=self._droneID).join()
            self._client.hoverAsync(vehicle_name=self._droneID).join()

    def waypoint_flight(self, check_boundary=2):
        """
        Agent command drone to fly to specific target position
        follow global waypoint
        """
        self.check_path3D(check_boundary=check_boundary)
        if self.check_end3D(check_boundary=check_boundary):
            return vector.Vector()

        steer = vector.Vector() + self._global_path_list[self._path_index] - self._location # speed
        steer = steer.normalize() * self._maxspeed
        steer = steer - self._velocity  # force
        steer = steer.make_steer(self._maxforce)

        return steer
    
    def collision_avoidance(self, weight=1, drones=[], visible=[], height_control=True):
        """
        Seperate Rule
        Calculate vector for the rule of collision avoidance
        weight: the weight for the rule of collision avoidance
        height_control: if height_control is True --> it works in 2D
        """
        steer = vector.Vector()
        vec_sum = vector.Vector()
        count = 0
        if height_control:
            for i in range(len(drones)):
                if visible[i]:
                    distance = localmap.distance2Dv(loc2d1=self._location, loc2d2=drones[i]['location'])
                    if distance <= self._object_boundary:
                        count += 1
                        difference = vector.Vector() + self._location - drones[i]['location']
                        vec_sum += difference.normalize2D() * self._maximum_avoidance_coefficient
                    elif distance < self._minimum_avodance_boundary:
                        count += 1
                        difference = vector.Vector() + self._location - drones[i]['location']
                        coefficient = ((1 - self._maximum_avoidance_coefficient) * distance + self._minimum_avodance_boundary * self._maximum_avoidance_coefficient - 1) / (self._minimum_avodance_boundary - 1)
                        vec_sum += difference.normalize2D() * coefficient
                    elif distance < self._seperation_boundary:
                        count += 1
                        difference = vector.Vector() + self._location - drones[i]['location']
                        vec_sum += difference.normalize2D() / (distance - self._minimum_avodance_boundary + 1)

            if count is 0:
                return steer
            
            steer = vec_sum / count # speed
            steer = steer.normalize2D() * self._maxspeed
            steer = steer - self._velocity  # force
            steer = steer.make_steer(self._maxforce)

            return steer * weight
        else:
            for i in range(len(drones)):
                if visible[i]:
                    distance = localmap.distance3Dv(loc3d1=self._location, loc3d2=drones[i]['location'])
                    if distance <= self._object_boundary:
                        count += 1
                        difference = vector.Vector() + self._location - drones[i]['location']
                        vec_sum += difference.normalize() * self._maximum_avoidance_coefficient
                    elif distance < self._minimum_avodance_boundary:
                        count += 1
                        difference = vector.Vector() + self._location - drones[i]['location']
                        coefficient = ((1 - self._maximum_avoidance_coefficient) * distance + self._minimum_avodance_boundary * self._maximum_avoidance_coefficient - 1) / (self._minimum_avodance_boundary - 1)
                        vec_sum += difference.normalize() * coefficient
                    elif distance < self._seperation_boundary:
                        count += 1
                        difference = vector.Vector() + self._location - drones[i]['location']
                        vec_sum += difference.normalize() / (distance - self._minimum_avodance_boundary + 1)

            if count is 0:
                return steer
                
            steer = vec_sum / count # speed
            steer = steer.normalize() * self._maxspeed
            steer = steer - self._velocity  # force
            steer = steer.make_steer(self._maxforce)

            return steer * weight

    def velocity_matching(self, weight=1, drones=[], visible=[], height_control=True):
        """
        Align Rule
        Calculate vector for the rule of velocity matching
        if leader, then agent fly with global path
        weight: the weight for the rule of velocity matching
        """
        steer = vector.Vector()
        vel_sum = vector.Vector()
        count = 0
        
        for i in range(len(drones)):
            # if visible[i]:
            if visible[i] and drones[i]['leader']:
                count += 1
                vel_sum += (vector.Vector() + drones[i]['velocity'])

        if count is 0:
            return steer

        if height_control:
            vel_sum.z_val = 0
        
        steer = vel_sum / count # speed
        steer = steer.normalize() * self._maxspeed
        steer = steer - self._velocity  # force
        steer = steer.make_steer(self._maxforce)

        return steer * weight

    def flocking_center(self, weight=1, drones=[], visible=[], height_control=True):
        """
        Cohesion Rule
        Calculate vector for the rule of flocking center
        weight: the weight for the rule of flocking center
        gpses: the gpses of other drones
        """
        steer = vector.Vector()
        center = vector.Vector()
        count = 0
        leader_location = vector.Vector()
        leader_alive = False

        for i in range(len(drones)):
            if visible[i]:
                count += 1
                center += drones[i]['location']
            if drones[i]['leader']:
                # count += len(drones)-1
                # center += drones[i]['location'] * (len(drones)-1)
                if height_control:
                    leader_location = drones[i]['location']
                    leader_alive = True

        if count is 0:
            return steer
            
        center /= count
        desired = vector.Vector() + center - self._location # speed
        if height_control and leader_alive:
            desired.z_val = leader_location.z_val - self._location.z_val
        
        steer = steer.normalize() * self._maxspeed
        steer = desired - self._velocity    # force
        steer.make_steer(self._maxforce)
    
        return steer * weight

    def flocking_flight(self, weights=[1, 1, 1], check_boundary=2, height_control=True):
        """
        Agent command drone to fly by flocking
        """
        if self._SITL:
            # collect drone's location
            self._location = self._client.getMultirotorState(vehicle_name=self._droneID).kinematics_estimated.position
            self._location.x_val += self._start_location[0]
            self._location.y_val += self._start_location[1]
            self._location.z_val += self._start_location[2]
            # self._angular_velocity = self._client.getMultirotorState().kinematics_estimated.angular_velocity

            self._location = self.gps_error(self._location)

            self._conn.send({'droneID': self._droneID, 'leader': self._leader, 'location': self._location, 'velocity': self._velocity})
            self.lte_delay()

            data = self._conn.recv()
            self.lte_delay()

            locations = []
            velocities = []
            if data[0] == 'broking':
                for data_dict in data[1]:
                    locations.append(data_dict['location'])
                    velocities.append(data_dict['velocity'])

            # check distance is less than boundary
            visible = []
            
            for location in locations:
                distance = localmap.distance3Dv(self._location, location)
                
                ### check angle also
                if distance < self._neighbor_boundary and distance > 0:
                    visible.append(True)
                else:
                    visible.append(False)

            if self._path_index >= len(self._global_path_list):
                self._client.hoverAsync(vehicle_name=self._droneID)
            elif self._leader:
                # leader waypoint flight
                self._maxspeed = self._global_velocity_list[self._path_index]

                acceleration = self.waypoint_flight(check_boundary=check_boundary)
                steer = self._velocity + acceleration
                steer = steer.make_steer(self._maxspeed)
                self._client.moveByVelocityAsync(steer.x_val, steer.y_val, steer.z_val, self._duration, vehicle_name=self._droneID)

                self._velocity = steer
                log_location = vector.Vector() + self._location
                log_velocity = vector.Vector() + self._velocity
                self._log.write('log_location: '+log_location.toString()+'\n')
                self._log.write('log_velocity: '+log_velocity.toString()+'\n\n')
            else:
                self._maxspeed = self._global_velocity_list[self._path_index]
                self._maxspeed *= self._maxspeed_weight

                col_avo = self.collision_avoidance(weight=weights[0], drones=data[1], visible=visible, height_control=height_control)
                vel_mat = self.velocity_matching(weight=weights[1], drones=data[1], visible=visible, height_control=height_control)
                flo_cet = self.flocking_center(weight=weights[2], drones=data[1], visible=visible, height_control=height_control)
                acceleration = col_avo + vel_mat + flo_cet
                steer = self._velocity + acceleration
                steer.make_steer(self._maxspeed)
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

    def formation_control(self, weight=1, drones=[dict()], mode='column', swarm_distance=5, height_control=True):
        if mode == 'column':
            return self.column_formation_control(weight=weight, drones=drones, swarm_distance=swarm_distance)
        elif mode == 'line':
            return self.line_formation_control(weight=weight, drones=drones, swarm_distance=swarm_distance)
        else:
            return vector.Vector()

    def column_formation_control(self, weight=1, drones=[dict()], swarm_distance=3):
        count = 0
        left_column = []
        right_column = []

        for i in range(len(drones)):
            drones[i]['distance'] = math.inf

        # find leader location & direction & velocity
        leader_location = vector.Vector()
        leader_direction = vector.Vector()
        for drone in drones:
            if drone['leader']:
                leader_location += drone['location']
                leader_direction += self._global_path_list[self._path_index] - leader_location
        
        leader_direction.normalize2D()

        # Y = aX + b, a = N/E, N: x_val, E: y_val
        if leader_direction.y_val == 0:
            a = math.inf * leader_direction.x_val
        else:
            a = leader_direction.x_val / leader_direction.y_val
        b = leader_location.x_val - a * leader_location.y_val

        # asign left or right
        for i in range(len(drones)):
            drones[i]['distance'] = localmap.distance3Dv(loc3d1=drones[i]['location'], loc3d2=leader_location)
            if not drones[i]['leader']:
                if leader_direction.y_val > 0:
                    if (drones[i]['location'].x_val - a * drones[i]['location'].y_val) >= b:
                        left_column.append(drones[i])
                    else:
                        right_column.append(drones[i])
                else:
                    if (drones[i]['location'].x_val - a * drones[i]['location'].y_val) <= b:
                        left_column.append(drones[i])
                    else:
                        right_column.append(drones[i])

        left_column = self.distance_qsort(left_column)
        right_column = self.distance_qsort(right_column)
        
        while len(left_column) > len(drones)/2:
            right_column.insert(0, left_column[0])
            left_column = left_column[1:]
            
        while len(right_column) > len(drones)/2:
            left_column.insert(0, right_column[0])
            right_column = right_column[1:]


        count = 0
        for i in range(len(left_column)):
            count += 1
            if left_column[i]['droneID'] == self._droneID:
                distribute = leader_direction.turn_left() * count * swarm_distance
        count = 0
        for i in range(len(right_column)):
            count += 1
            if right_column[i]['droneID'] == self._droneID:
                distribute = leader_direction.turn_right() * count * swarm_distance

        desired = distribute + leader_location - self._location # speed
        desired = desired.normalize() * self._maxspeed
        steer = desired - self._velocity    # force
        steer = steer.make_steer(self._maxforce)

        return steer * weight

    def line_formation_control(self, weight=1, drones=[dict()], swarm_distance=2):
        count = 0

        for i in range(len(drones)):
            drones[i]['distance'] = math.inf

        # find leader location & direction & velocity
        leader_location = vector.Vector()
        leader_direction = vector.Vector()
        for drone in drones:
            if drone['leader']:
                leader_location += drone['location']
                leader_direction += self._global_path_list[self._path_index] - leader_location
        
        leader_direction.normalize2D()

        # asign left or right
        for i in range(len(drones)):
            drones[i]['distance'] = localmap.distance3Dv(loc3d1=drones[i]['location'], loc3d2=leader_location)

        drones = self.distance_qsort(drones)

        count = 0
        for i in range(len(drones)):
            count += 1
            if drones[i]['droneID'] == self._droneID:
                distribute = leader_direction.turn_around() * count * swarm_distance

        desired = distribute + leader_location - self._location # speed
        desired = desired.normalize() * self._maxspeed
        steer = desired - self._velocity    # force
        steer = steer.make_steer(self._maxforce)

        return steer * weight

    def formation_flight(self, weights=[1, 1, 1], check_boundary=2, height_control=True, mode='column'):
        """
        Agent command drone to fly by formation
        weights: for flocking algorithm [seperate, align, cohesion]
        check_boundary: check_boundary for path checking
        height_control: arrange drone 2D or 3D
        mode: formation_flight mode
            'column'
            'line'
        """
        if self._SITL:
            # collect drone's location
            self._location = self._client.getMultirotorState(vehicle_name=self._droneID).kinematics_estimated.position
            self._location.x_val += self._start_location[0]
            self._location.y_val += self._start_location[1]
            self._location.z_val += self._start_location[2]
            # self._angular_velocity = self._client.getMultirotorState().kinematics_estimated.angular_velocity

            self._location = self.gps_error(self._location)

            self._conn.send({'droneID': self._droneID, 'leader': self._leader, 'location': self._location, 'velocity': self._velocity})
            self.lte_delay()

            data = self._conn.recv()
            self.lte_delay()
            
            locations = []
            velocities = []
            if data[0] == 'broking':
                for data_dict in data[1]:
                    locations.append(data_dict['location'])
                    velocities.append(data_dict['velocity'])

            # check distance is less than boundary
            visible = []
            
            for location in locations:
                distance = localmap.distance3Dv(self._location, location)
                
                ### check angle also
                if distance < self._neighbor_boundary and distance > 0:
                    visible.append(True)
                else:
                    visible.append(False)

            if self._path_index >= len(self._global_path_list):
                self._client.hoverAsync(vehicle_name=self._droneID)
            elif self._leader:
                # leader waypoint flight
                self._maxspeed = self._global_velocity_list[self._path_index]
                
                acceleration = self.waypoint_flight(check_boundary=check_boundary)
                steer = self._velocity + acceleration
                steer.make_steer(self._maxspeed)
                self._client.moveByVelocityAsync(steer.x_val, steer.y_val, steer.z_val, self._duration, vehicle_name=self._droneID)

                self._velocity = steer
                log_location = vector.Vector() + self._location
                log_velocity = vector.Vector() + self._velocity
                self._log.write('log_location: '+log_location.toString()+'\n')
                self._log.write('log_velocity: '+log_velocity.toString()+'\n\n')
            else:
                self._maxspeed = self._global_velocity_list[self._path_index]
                self._maxspeed *= self._maxspeed_weight

                col_avo = self.collision_avoidance(weight=weights[0], drones=data[1], visible=visible, height_control=height_control)
                vel_mat = self.velocity_matching(weight=weights[1], drones=data[1], visible=visible, height_control=height_control)
                for_con = self.formation_control(weight=weights[2], drones=data[1], mode=mode, height_control=height_control)
                acceleration = col_avo + vel_mat + for_con
                steer = self._velocity + acceleration
                steer.make_steer(self._maxspeed)
                self._client.moveByVelocityAsync(steer.x_val, steer.y_val, steer.z_val, self._duration, vehicle_name=self._droneID)

                self._velocity = steer
                log_location = vector.Vector() + self._location
                log_velocity = vector.Vector() + self._velocity
                self._log.write('col_avo: '+col_avo.toString()+'\n')
                self._log.write('steer: '+steer.toString()+'\n')
                self._log.write('log_location: '+log_location.toString()+'\n')
                self._log.write('log_velocity: '+log_velocity.toString()+'\n\n')
        else:
            pass

    def collect_data(self):
        """
        Collect Agent's data
        """
        self._location = self._client.getMultirotorState(vehicle_name=self._droneID).kinematics_estimated.position
        self._location.x_val += self._start_location[0]
        self._location.y_val += self._start_location[1]
        self._location.z_val += self._start_location[2]
        self._location = self.gps_error(self._location)
        self._conn.send([self._droneID, self._location])

    def distance_qsort(self, inlist=[]):
        if inlist == []:
            return []
        else:
            pivot = inlist[0]
            lesser = self.distance_qsort([x for x in inlist[1:] if x['distance'] < pivot['distance']])
            greater = self.distance_qsort([x for x in inlist[1:] if x['distance'] >= pivot['distance']])
            return lesser + [pivot] + greater

    # def agent_controller(self):
    #     self._socket.connect((self._host, self._port))
    #     self._socket.send(b'Hello, python')
    #     data = self._socket.recv(1024)
    #     self._socket.close()
    #     print('Received', repr(data))

def run_agent(conn, leader=True, SITL=True, droneID='', start_location=[0, 0, 0], seperation_boundary=2, neighbor_boundary=5, neighbor_angle=180, local_map=localmap.LocalMap(coords=[], SITL=True)):
    """
    run drone agent with gcs command for Unreal Engine with AirSim
    conn: child_conn of Pipe() to connection with GCS
    leader: Initialize agent's role
        True: leader
        False: follower
    SITL: Initialize environment is Unreal Engine or ROS
        True: Unreal Engine
        False: ROS
    client: Initialize client if environment is Unreal Engine
    droneID: Initialize drone's id
    distance: Initialize agent's flocking neighborhood boundary distance(m)
    angle: Initialize agent's flocking neighborhood boundary angle(0~180)
    local_map: map of specific gps area
    """
    droneAgent = DroneAgent(leader=leader, SITL=SITL, conn=conn, droneID=droneID, seperation_boundary=seperation_boundary, neighbor_boundary=neighbor_boundary, neighbor_angle=neighbor_angle, local_map=local_map, start_location=start_location)
    while True:
        if SITL:
            command = conn.recv()
            if command[0] == 'set_global_path':
                droneAgent.set_global_path(global_path_list=command[1], global_velocity_list=command[2])
            elif command[0] == 'collect_data':
                droneAgent.collect_data()
            elif command[0] == 'takeoff':
                droneAgent.takeoff()
            elif command[0] == 'flocking_flight':
                droneAgent.flocking_flight(weights=command[1], check_boundary=command[2])
            elif command[0] == 'formation_flight':
                droneAgent.formation_flight(weights=command[1], check_boundary=command[2], mode=command[3])
            elif command[0] == 'land':
                droneAgent.land()
            elif command[0] == 'end':
                break

if __name__ == "__main__":
    pass
    # agent = DroneAgent(leader=True, SITL=True, )