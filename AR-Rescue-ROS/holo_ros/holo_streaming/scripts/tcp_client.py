import socket
import struct
import time


class TcpClient(object):
    
    def __init__(self, ip, port):
        self.ip_ = ip
        self.port_ = port
        self.sock_ = None


    def send(self, data_bin):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect( (self.ip_, self.port_) )

        sock.sendall(data_bin)
        sock.close()