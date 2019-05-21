import socket
import json
from threading import Thread
import time
import struct
import sys

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

    """
    # Check: https://stackoverflow.com/questions/17667903/python-socket-receive-large-amount-of-data

    def recv_message(self, child_conn):
        self._conn, _ = self._s.accept()
        buffer = ''
        while True:
            msg = self._conn.recv(4096)
            msg = msg.decode('utf-8')
            sys.stdout.write("[%f] PipeServer :: recv_message: %s\n" % (time.time(), msg))

            if buffer != '':
                msg = buffer + msg
                buffer = ''
            msgs = msg.strip().split('\n')

            for i in range(len(msgs)):
                if i == len(msgs)-1 and msg[-1] != '\n':
                    buffer += msgs[i]
                msgs[i] = json.loads(msgs[i])
                child_conn.send(msgs[i])

    def send_message(self, child_conn):
        while self._conn is None:
            pass
        while True:
            datas = child_conn.recv()
            msg = json.dumps(datas)+'\n'
            self._conn.sendall(msg.encode('utf-8'))
    """

    def recv_message(self, child_conn):
        self._conn, _ = self._s.accept()
        while True:
            # Read message length and unpack it into an integer.
            rsize = self.recvall(4)
            if not rsize:
                raise Exception("Invalid message packet size: %r" % rsize)
                #return None
            size = struct.unpack('>I', rsize)[0]
            # Read the message data
            data = self.recvall(size)
            data = data.decode("utf-8")
            data = json.loads(data)
            sys.stdout.write("[%f] PipeClient :: recv_message: %s\n" % (time.time(), data))
            child_conn.send(data)

    def recvall(self, psize):
        # Helper method to receive "psize" bytes or return None if EOF is hit.
        data = b''
        while len(data) < psize:
            packet = self._conn.recv(psize-len(data))
            if not packet:
                return None # EOF
            data += packet
        return data

    def send_message(self, child_conn):
        while self._conn is None:
            pass
        while True:
            try:
                data = child_conn.recv()
                message = json.dumps(data) + '\n'
                message = message.encode("utf-8")
                # Prefix each message with a 4-byte length (network byte order).
                message = struct.pack('>I', len(message)) + message
                self._conn.sendall(message)
            except EOFError as e:
                sys.stderr.write("[%f] PipeClient :: send_message : %s\n" % (time.time(), e))

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
        print('host: %s, port: %d' % (host, port))
        self._s.connect((host, port))

        self._recv_proc = Thread(target=self.recv_message, args=(child_conn,))
        self._send_proc = Thread(target=self.send_message, args=(child_conn,))

    def start(self):
        self._recv_proc.start()
        self._send_proc.start()

    """
    # Check: https://stackoverflow.com/questions/17667903/python-socket-receive-large-amount-of-data

    def recv_message(self, child_conn):
        buffer = ''
        while True:
            msg = self._s.recv(4096)
            msg = msg.decode('utf-8')
            sys.stdout.write("[%f] PipeClient :: recv_message: %s\n" % (time.time(), msg))

            if buffer != '':
                msg = buffer + msg
                buffer = ''
            msgs = msg.strip().split('\n')

            for i in range(len(msgs)):
                if i == len(msgs)-1 and msg[-1] != '\n':
                    buffer += msgs[i]
                msgs[i] = json.loads(msgs[i])
                child_conn.send(msgs[i])

    def send_message(self, child_conn):
        while True:
            try:
                datas = child_conn.recv()
                msg = json.dumps(datas)+'\n'
                self._s.sendall(msg.encode('utf-8'))
            except EOFError as e:
                print(e)

    """

    def recv_message(self, child_conn):
        while True:
            # Read message length and unpack it into an integer.
            rsize = self.recvall(4)
            if not rsize:
                raise Exception("Invalid message packet size: %r" % rsize)
                #return None
            size = struct.unpack('>I', rsize)[0]
            # Read the message data
            data = self.recvall(size)
            data = data.decode("utf-8")
            data = json.loads(data)
            sys.stdout.write("[%f] PipeClient :: recv_message: %s\n" % (time.time(), data))
            child_conn.send(data)

    def recvall(self, psize):
        # Helper method to receive "psize" bytes or return None if EOF is hit.
        data = b''
        while len(data) < psize:
            packet = self._s.recv(psize-len(data))
            if not packet:
                return None # EOF
            data += packet
        return data

    def send_message(self, child_conn):
        while True:
            try:
                data = child_conn.recv()
                message = json.dumps(data) + '\n'
                message = message.encode("utf-8")
                # Prefix each message with a 4-byte length (network byte order).
                message = struct.pack('>I', len(message)) + message
                self._s.sendall(message)
            except EOFError as e:
                sys.stderr.write("[%f] PipeClient :: send_message : %s\n" % (time.time(), e))

    def close(self):
        self._recv_proc.join()
        self._send_proc.join()

if __name__ is '__main__':
    from multiprocessing import Pipe

    parent, child = Pipe()

    proc = PipeServer(child, '127.0.0.1', 4000)

    proc.start()

    while True:
        data = parent.recv()
        print(data)
        parent.send(data)
