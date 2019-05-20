import json
from multiprocessing import Process, Pipe
from pipesocket import PipeServer

def main():
    droneIDs=['Drone1', 'Drone2', 'Drone3', 'Drone4', 'Drone5', 'Drone6', 'Drone7', 'Drone8', 'Drone9']
    is_leader=[True, False, False, False, False, False, False, False, False]
    errors=[[0, 0, 0], [2, 2, 0], [4, 0, 0], [2, -2, 0], [2, 0, 0], [4, -2, 0], [4, 2, 0], [0, -2, 0], [0, 2, 0]]

    processes = []
    parentConn = []

    host = '127.0.0.1'
    client_port = 4000
    drone_port = [client_port + i for i in range(1, 10)]

    clientParent, clientChild = Pipe()
    clientProc = PipeServer(clientChild, host, client_port)
    clientProc.start()

    num = clientParent.recv()

    for i in range(num):
        parent, child = Pipe()
        proc = PipeServer(child, host, drone_port[i])
        proc.start()
        processes.append(proc)
        parentConn.append(parent)

    for i in range(num):
        datas = dict()
        datas['droneID'] = droneIDs[i]
        datas['is_leader'] = is_leader[i]
        datas['error'] = errors[i]
        parentConn[i].send(datas)

    while True:
        msg_dict = clientParent.recv()
        print(msg_dict['command'])

        if msg_dict['command'] == 'end':
            for i in range(num):
                parentConn[i].send(msg_dict)
                processes[i].close()
            return
        elif msg_dict['command'] == 'takeoff':
            for i in range(num):
                parentConn[i].send(msg_dict)
        elif msg_dict['command'] == 'land':
            for i in range(num):
                parentConn[i].send(msg_dict)
        elif msg_dict['command'] == 'set_global_path':
            for i in range(num):
                parentConn[i].send(msg_dict)
        elif msg_dict['command'] == 'flocking_flight':
            for i in range(num):
                parentConn[i].send(msg_dict)
        elif msg_dict['command'] == 'formation_flight':
            for i in range(num):
                parentConn[i].send(msg_dict)
        elif msg_dict['command'] == 'broking':
            datas = []
            for i in range(num):
                datas.append(parentConn[i].recv())
            for i in range(num):
                parentConn[i].send(datas)
        elif msg_dict['command'] == 'collect_data':
            for i in range(num):
                parentConn[i].send(msg_dict)
            datas = []
            for i in range(num):
                datas.append(parentConn[i].recv())
            clientParent.send(datas)

if __name__ == '__main__':
    main()