from agent import DroneAgent
from vector import Vector
import localmap
import haversine
from multiprocessing import Process, Pipe

class GCS:
    """
    Ground Control System
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
            self._droneConns.append(Pipe())
            self._processes.append(DroneAgent(leader=self._is_leader[i], conns=self._droneConns, UE=True, droneID=droneIDs[i]).run_agent())

    def start(self):
        for i in range(len(self._droneIDs)):
            self._processes[i].start()

    def join(self):
        for i in range(len(self._droneIDs)):
            self._processes[i].join()

    def send_command(self, command=''):
        for i in range(len(self._droneIDs)):
            self._droneConns[i][0].send(command)

def __main__():
    control = GCS(droneIDs=['Drone1', 'Drone2', 'Drone3'], is_leader=[True, False, False])
    # start
    control.start()

    # control test

    # join
    control.join()