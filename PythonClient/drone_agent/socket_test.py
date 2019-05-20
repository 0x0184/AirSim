from multiprocessing import Pipe
from pipesocket import PipeClient

parent, child = Pipe()

proc = PipeClient(child, '127.0.0.1', 4000)

proc.start()

for i in range(100):
    parent.send({'droneID': 'drone'+str(i), 'leader': False, 'location': [0, 0, 0], 'velocity': [0, 0, 0]})

for i in range(100):
    print(parent.recv())