import agent
from vector import Vector
import localmap
import haversine
from multiprocessing import Process, Pipe
import time

class SITL:
    """
    Ground Control System for Unreal Engine with AirSim
    setting.json must be set like this
    {
        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ClockSpeed": 1,
        
        "Vehicles": {
            "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": 0, "Y": 0, "Z": 0
            },
            "Drone2": {
            "VehicleType": "SimpleFlight",
            "X": 2, "Y": 2, "Z": 0
            },
            "Drone3": {
            "VehicleType": "SimpleFlight",
            "X": 4, "Y": 0, "Z": 0
            }

        }
    }
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
            proc = Process(target=agent.run_agent, args=(self._droneCConns[i], self._is_leader[i], True, droneIDs[i], self._error[i], 25, 50))
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
                if distance < boundary:
                    self._path_index += 1
                if self._path_index >= len(path_list):
                    self._path_index = 0
                    return True
        return False

if __name__ is '__main__':
    control = SITL(droneIDs=['Drone1', 'Drone2', 'Drone3', 'Drone4', 'Drone5', 'Drone6', 'Drone7', 'Drone8', 'Drone9'], is_leader=[True, False, False, False, False, False, False, False, False], error=[[0, 0, 0], [2, 2, 0], [4, 0, 0], [2, -2, 0], [2, 0, 0], [4, -2, 0], [4, 2, 0], [0, -2, 0], [0, 2, 0]])
    
    # start
    control.start()

    # set path list
    path_list1 = [Vector(0, 100, -15), Vector(0, 200, -30), Vector(0, 300, -30)]
    speed_list1 = [10, 10, 10]
    path_list2 = [Vector(300, 300, -30)]
    speed_list2 = [10]
    path_list3 = [Vector(300, 0, -30)]
    speed_list3 = [10]
    path_list4 = [Vector(200, 0, -30), Vector(100, 0, -30), Vector(0, 0, -30)]
    speed_list4 = [10, 10, 10]
    check_boundary = 2
    mission_boundary = [1]
    flocking_boundary = [25]

    # control test
    control.send_command('takeoff')
    
    datas = control.send_command('collect_data')
    control.send_command('set_global_path', [path_list1, speed_list1])
    while not control.mission_complete(datas, path_list1, boundary=3):
        control.send_command('flocking_flight', data=[[1.5, 1, 1.5], check_boundary])
        control.broking()
        datas = control.send_command('collect_data')
    control.send_command('set_global_path', [path_list2, speed_list2])
    while not control.mission_complete(datas, path_list2, boundary=3):
        control.send_command('formation_flight', data=[[1.5, 1.5, 15], check_boundary, 'column'])
        control.broking()
        datas = control.send_command('collect_data')
    control.send_command('set_global_path', [path_list3, speed_list3])
    while not control.mission_complete(datas, path_list3, boundary=3):
        control.send_command('formation_flight', data=[[1.5, 1, 15], check_boundary, 'line'])
        control.broking()
        datas = control.send_command('collect_data')
    control.send_command('set_global_path', [path_list4, speed_list4])
    while not control.mission_complete(datas, path_list4, boundary=3):
        control.send_command('flocking_flight', data=[[1.5, 1.5, 1.5], check_boundary])
        control.broking()
        datas = control.send_command('collect_data')

    control.send_command('land')

    # join
    control.send_command('end')
    control.join()