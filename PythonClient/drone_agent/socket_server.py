from multiprocessing import Process, Pipe
import socket
import json

def broadcasting(child_pipe, port=4000):
    host = ''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        while True:
            s.listen(1)
            conn, addr = s.accept()
            msg = conn.recv(1024)
            msg = msg.decode('utf-8')
            msg = json.loads(msg)
            child_pipe.send(msg)
            msg = child_pipe.recv()
            msg = json.dumps(msg)
            conn.sendall(msg)
        conn.close()

def main(num=9):
    processes = []
    dronePConns = []
    droneCConns = []

    for i in range(num):
        parrent_conn, child_conn = Pipe()
        dronePConns.append(parrent_conn)
        droneCConns.append(child_conn)

    for i in range(num):
        proc = Process(target=broadcasting, args=(droneCConns[i], 4000+i))
        processes.append(proc)

    for i in range(num):
        processes[i].start()

if __name__ == '__main__':
    main(9)