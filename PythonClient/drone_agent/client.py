import agent
from vector import Vector
import localmap
import haversine
import time
import socket
import json
from pipesocket import PipeClient
from multiprocessing import Pipe

path_index = 0

def mission_complete(datas, path_list, boundary=3):
    global path_index

    for i in range(len(datas)):
        if datas[i]['is_leader']:
            leader_location = datas[i]['location']
            distance = localmap.distance3Dv(loc3d1=leader_location, loc3d2=path_list[path_index])
            if distance < boundary:
                path_index += 1
            if path_index >= len(path_list):
                path_index = 0
                return True
    return False

if __name__ is '__main__':
    host = '127.0.0.1'
    port = 4000
    drone_num = 9

    # set path list
    path_list1 = [Vector(0, 100, -15), Vector(0, 200, -40), Vector(0, 300, -40)]
    speed_list1 = [10, 10, 10]
    path_list2 = [Vector(200, 300, -40)]
    speed_list2 = [10]
    path_list3 = [Vector(400, 300, -40)]
    speed_list3 = [10]
    path_list4 = [Vector(200, 0, -40), Vector(100, 0, -40), Vector(0, 0, -40)]
    speed_list4 = [10, 10, 10]
    check_boundary = 2
    mission_boundary = [1]
    flocking_boundary = [25]

    parent, child = Pipe()
    proc = PipeClient(child, host, port)
    proc.start()

    parent.send(drone_num)

    input('start_simulation')

    command = dict()

    command['command'] = 'takeoff'
    parent.send(command)

    command['command'] = 'collect_data'
    parent.send(command)
    datas = parent.recv()

    command['command'] = 'set_global_path'
    command['data'] = [path_list1, speed_list1]
    parent.send(command)

    while not mission_complete(datas, path_list1, boundary=3):
        command['command'] = 'flocking_flight'
        command['data'] = [[1.5, 1, 1], check_boundary]
        parent.send(command)

        command['command'] = 'broking'
        parent.send(command)

        command['command'] = 'collect_data'
        parent.send(command)
        datas = parent.recv()

    command['command'] = 'set_global_path'
    command['data'] = [path_list2, speed_list2]
    parent.send(command)

    while not mission_complete(datas, path_list1, boundary=3):
        command['command'] = 'formation_flight'
        command['data'] = [[1, 1, 5], check_boundary, 'column']
        parent.send(command)

        command['command'] = 'broking'
        parent.send(command)

        command['command'] = 'collect_data'
        parent.send(command)
        datas = parent.recv()

    command['command'] = 'set_global_path'
    command['data'] = [path_list3, speed_list3]
    parent.send(command)

    while not mission_complete(datas, path_list1, boundary=3):
        command['command'] = 'formation_flight'
        command['data'] = [[1, 1, 5], check_boundary, 'line']
        parent.send(command)

        command['command'] = 'broking'
        parent.send(command)

        command['command'] = 'collect_data'
        parent.send(command)
        datas = parent.recv()

    command['command'] = 'set_global_path'
    command['data'] = [path_list4, speed_list4]
    parent.send(command)

    while not mission_complete(datas, path_list1, boundary=3):
        command['command'] = 'flocking_flight'
        command['data'] = [[1.5, 1, 1], check_boundary]
        parent.send(command)

        command['command'] = 'broking'
        parent.send(command)

        command['command'] = 'collect_data'
        parent.send(command)
        datas = parent.recv()

    command['command'] = 'land'
    parent.send(command)

    command['command'] = 'end'
    parent.send(command)