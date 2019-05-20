import socket
import json
from threading import Thread
import time

class PipeServer:
    """
    Socket Server similiar to multiprocessing.Pipe
    """
    def __init__(self, child_conn, host='', port=4000):
        self._s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._s.bind((host, port))
        self._s.listen(1)
        self._conn = None

        self._recv_proc = Thread(target=self.recv_message, args=(child_conn,))
        self._send_proc = Thread(target=self.send_message, args=(child_conn,))

    def start(self):
        self._recv_proc.start()
        self._send_proc.start()

    def recv_message(self, child_conn):
        self._conn, _ = self._s.accept()
        while True:
            msg = self._conn.recv(1024)
            msg = msg.decode('utf-8')
            msg = json.loads(msg)
            child_conn.send(msg)

    def send_message(self, child_conn):
        while self._conn is None:
            pass
        while True:
            datas = child_conn.recv()
            msg = json.dumps(datas)
            self._conn.sendall(msg.encode('utf-8'))
            time.sleep(0.01)

    def close(self):
        self._recv_proc.join()
        self._send_proc.join()
        self._conn.close()

        
class PipeClient:
    """
    Socket Client similiar to multiprocessing.Pipe
    """
    def __init__(self, child_conn, host='', port=4000):
        self._s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._s.connect((host, port))

        self._recv_proc = Thread(target=self.recv_message, args=(child_conn,))
        self._send_proc = Thread(target=self.send_message, args=(child_conn,))

    def start(self):
        self._recv_proc.start()
        self._send_proc.start()

    def recv_message(self, child_conn):
        while True:
            msg = self._s.recv(1024)
            msg = msg.decode('utf-8')
            msg = json.loads(msg)
            child_conn.send(msg)

    def send_message(self, child_conn):
        while True:
            datas = child_conn.recv()
            msg = json.dumps(datas)
            self._s.sendall(msg.encode('utf-8'))
            time.sleep(0.01)

    def close(self):
        self._recv_proc.join()
        self._send_proc.join()

if __name__ is '__main__':
    from multiprocessing import Pipe

    parent, child = Pipe()

    proc = PipeServer(child, '192.168.0.54', 4000)

    proc.start()

    while True:
        data = parent.recv()
        print(data)
        parent.send(data)
