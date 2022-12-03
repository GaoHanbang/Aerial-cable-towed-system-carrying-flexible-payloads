#!/usr/bin/env python3

# TODO: Node part todo, only printing data for now
import socket
import time
from datetime import datetime
import sys
import csv

UDP_IP_ADDRESS = "192.168.0.14" # Your address, can be detected automatically?
UDP_PORT_NO = 2390
serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverSock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))

log_path = '/home/user/logs/fpr/sensor/'
header = ['timestamp', 'data']

def main():
    write_to_csv = False
    if len(sys.argv[1:]) == 1 and sys.argv[1] == 'record':
        now = datetime.now()
        dt_string = now.strftime("%Y-%m-%d_%H-%M-%S")
        f = open(log_path+dt_string+'.csv', 'w', encoding='UTF8')
        writer = csv.writer(f)
        writer.writerow(header)
        write_to_csv = True

    while True:
        try:
            serverSock.sendto(bytes(1), ('192.168.0.20', UDP_PORT_NO))
            data, addr = serverSock.recvfrom(1024)
            val = float(data.decode("ASCII"))
            print ("Message: %.1f" % val)
            if write_to_csv:
                row = [time.time(), val]
                writer.writerow(row)
            #time.sleep(0.03)
        except KeyboardInterrupt:
            print("Interrupted by user, program exits")
            if write_to_csv:
                f.close()
            exit(1)

if __name__ == '__main__':
    main()
