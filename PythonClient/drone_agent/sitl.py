import agent
from vector import Vector
import localmap
import haversine
from multiprocessing import Process, Pipe
import time

class SITL:
    """
    Ground Control System for Unreal Engine with AirSim
    setting.json must be set like setting_examples
    """
    def __init__(self, droneIDs=['Drone1'], is_leader=[True], error=None):
        # setting all
        self._processes = []
        self._dronePConns = []
        self._droneCConns = []
        self._droneIDs = droneIDs
        self._is_leader = is_leader
        self._leaderID = ''
        self._error = error
        self._path_index = 0
        for i in range(len(self._droneIDs)):
            ### need to consider Drone and GCS's pipe connection
            ### is 'broadcast' or '1 to 1 data transfer' or 'server and client' method
            parent_conn, child_conn = Pipe()
            self._dronePConns.append(parent_conn)
            self._droneCConns.append(child_conn)

        for i in range(len(self._droneIDs)):
            proc = Process(target=agent.run_agent, args=(self._droneCConns[i], self._is_leader[i], True, droneIDs[i], self._error[i], 10, 50))
            self._processes.append(proc)
            if self._is_leader[i]:
                self._leaderID = self._droneIDs[i]

    def start(self):
        for i in range(len(self._droneIDs)):
            self._processes[i].start()

    def join(self):
        for i in range(len(self._droneIDs)):
            self._processes[i].join()

    def send_command(self, command='', data=[]):
        for i in range(len(self._droneIDs)):
            self._dronePConns[i].send([command]+data)
        
        if command is 'collect_data':
            datas = []
            for i in range(len(self._droneIDs)):
                data = self._dronePConns[i].recv()
                # print(f'{self._droneIDs[i]} : {data}')
                datas.append(data)
            # print()
            return datas

    def broking(self):
        data = []
        for i in range(len(self._droneIDs)):
            data.append(self._dronePConns[i].recv())

        for i in range(len(self._droneIDs)):
            self._dronePConns[i].send(['broking', data])

    def mission_complete(self, datas, path_list, boundary=3):
        for i in range(len(datas)):
            if datas[i][0] == self._leaderID:
                leader_location = datas[i][1]
                distance = localmap.distance3Dv(loc3d1=leader_location, loc3d2=path_list[self._path_index])
                if distance <= boundary:
                    self._path_index += 1
                if self._path_index >= len(path_list):
                    self._path_index = 0
                    return True
        return False

if __name__ is '__main__':
    control = SITL(droneIDs=['Drone1', 'Drone2', 'Drone3'], is_leader=[True, False, False], error=[[0, 0, 0], [0, 2, 0], [0, -2, 0]])
    
    # start
    control.start()

    # set path list
    path_list1 = [Vector(50, 0, -3)]
    speed_list1 = [1]
    path_list2 = [Vector(0, 0, -3)]
    speed_list2 = [1]
    path_list3 = [Vector(50, 0, -3)]
    speed_list3 = [1]
    path_list4 = [Vector(20, 0, -3)]
    speed_list4 = [1]
    check_boundary = 3
    flocking_boundary = [25]

    # control test
    control.send_command('takeoff')
    # weights
    # collision avoidance, velocity matching, flocking center
    
    datas = control.send_command('collect_data')
    control.send_command('set_global_path', [path_list1, speed_list1])
    while not control.mission_complete(datas, path_list1, boundary=check_boundary):
        control.send_command('flocking_flight', data=[[1.5, 1, 1], check_boundary])
        control.broking()
        datas = control.send_command('collect_data')
        print(datas)
        print('loop1')
    control.send_command('set_global_path', [path_list2, speed_list2])
    while not control.mission_complete(datas, path_list2, boundary=check_boundary):
        control.send_command('formation_flight', data=[[1.5, 1, 5], check_boundary, 'line'])
        control.broking()
        datas = control.send_command('collect_data')
        print(datas)
        print('loop2')
    control.send_command('set_global_path', [path_list3, speed_list3])
    while not control.mission_complete(datas, path_list3, boundary=check_boundary):
        control.send_command('formation_flight', data=[[1.5, 1, 5], check_boundary, 'column'])
        control.broking()
        datas = control.send_command('collect_data')
        print(datas)
        print('loop3')
    control.send_command('set_global_path', [path_list4, speed_list4])
    while not control.mission_complete(datas, path_list4, boundary=check_boundary):
        control.send_command('flocking_flight', data=[[1.5, 1, 1], check_boundary])
        control.broking()
        datas = control.send_command('collect_data')
        print(datas)
        print('loop4')

    control.send_command('land')

    # join
    control.send_command('end')
    control.join()