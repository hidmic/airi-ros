#!/usr/bin/env python

import msgpack
import socket
import sys
from threading import Thread

BUFFER_SIZE = 1024

DISCOVERY_ADDR = '10.10.10.2'

ULTRS1_PORT = 10001
QENCODER_PORT = 10002


def init_socket():
    socket_list = []
    for port in range (QENCODER_PORT, 10003):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto("Hello", (DISCOVERY_ADDR, port))
        socket_list.append(sock)
    return socket_list


def communicate_jd(socket_list):
    while(True):
        for socket in socket_list:
            data, addr = socket.recvfrom(BUFFER_SIZE)
            if data:
                print("Received from {}:{} - {}.".format(addr[0], addr[1], data))


if __name__ == "__main__":
    socket_list = init_socket()
    communicate_jd(socket_list)
    #thread = Thread(target=communicate_jd, args=(socket_list,))
    #thread.start()
    #thread.join()
