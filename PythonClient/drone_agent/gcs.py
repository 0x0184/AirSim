from agent import DroneAgent
from vector import Vector
import localmap
import haversine
from multiprocessing import Process, Pipe
import time

class GCS:
    """
    Ground Control System for Unreal Engine with AirSim
    """
    def __init__(self, droneIDs=['Drone1'], is_leader=[True]):
        # setting all
        self._processes = []
        self._droneConns = []
        self._droneIDs = droneIDs
        self._is_leader = is_leader

        for i in range(len(self._droneIDs)):
            ### need to consider Drone and GCS's pipe connection
            ### is 'broadcast' or '1 to 1 data transfer' or 'server and client' method
            parent_conn, child_conn = Pipe()
            self._droneConns.append(parent_conn)
            self._processes.append(DroneAgent(leader=self._is_leader[i], conn=child_conn, UE=True, droneID=droneIDs[i]).run_agent())

    def start(self):
        for i in range(len(self._droneIDs)):
            self._processes[i].start()

    def join(self):
        for i in range(len(self._droneIDs)):
            self._processes[i].join()

    def send_command(self, command='', data=None):
        for i in range(len(self._droneIDs)):
            self._droneConns[i].send([command]+data)
        
        if command is 'collect_data':
            datas = []
            for i in range(len(self._droneIDs)):
                data = self._droneConns[i].recv()
                print(f'{self._droneIDs[i]} : {data}')
                datas.append(data)
            print()
            return datas

    def broking(self):
        data = []
        for i in range(len(self._droneIDs)):
            data.append(self._droneConns[i].recv())

        for i in range(len(self._droneIDs)):
            self._droneConns[i].send(['broking', data])

if __name__ is '__main__':
    control = GCS(droneIDs=['Drone1', 'Drone2', 'Drone3'], is_leader=[True, False, False])
    # start
    control.start()

    # control test
    control.send_command('set_global_path', [[[100, 100, -10]], [[5]]])
    control.send_command('takeoff')
    # time.sleep(100)
    
    datas = control.send_command('collect_data')
    while not (datas[0][1].x_val is 100 and datas[0][1].y_val is 100, datas[0][1].z_val is -10):
        control.send_command('flocking_flight', data=[[1, 1, 1], 5])
        datas = control.send_command('collect_data')

    control.send_command('land')

    # join
    control.send_command('end')
    control.join()