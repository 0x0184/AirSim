import agent
from vector import Vector
import localmap
import haversine
from multiprocessing import Process, Pipe
import time

class GCS:
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
        self._error = error
        for i in range(len(self._droneIDs)):
            ### need to consider Drone and GCS's pipe connection
            ### is 'broadcast' or '1 to 1 data transfer' or 'server and client' method
            parent_conn, child_conn = Pipe()
            self._dronePConns.append(parent_conn)
            self._droneCConns.append(child_conn)


        for i in range(len(self._droneIDs)):
            proc = Process(target=agent.run_agent, args=(self._droneCConns[i], self._is_leader[i], True, droneIDs[i], self._error[i], ))
            self._processes.append(proc)


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
                print(f'{self._droneIDs[i]} : {data}')
                datas.append(data)
            print()
            return datas

    def broking(self):
        data = []
        for i in range(len(self._droneIDs)):
            data.append(self._dronePConns[i].recv())

        for i in range(len(self._droneIDs)):
            self._dronePConns[i].send(['broking', data])

if __name__ is '__main__':
    control = GCS(droneIDs=['Drone1', 'Drone2', 'Drone3'], is_leader=[True, False, False], error=[[0, 0, 0], [2, 2, 0], [4, 0, 0]])
    # start
    control.start()

    # control test
    control.send_command('set_global_path', [[[100, 100, -10]], [[5]]])
    control.send_command('takeoff')
    # time.sleep(100)
    
    datas = control.send_command('collect_data')
    while datas[0][1].x_val is not 100 or datas[0][1].y_val is not 100 or datas[0][1].z_val is not -10:
        control.send_command('flocking_flight', data=[[1, 1, 1], 5])
        control.broking()
        datas = control.send_command('collect_data')

    control.send_command('land')

    # join
    control.send_command('end')
    control.join()