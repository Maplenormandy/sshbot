#!/usr/bin/python
import socket, sys, time, base64, json

HOST=sys.argv[1]
PORT=sys.argv[2]
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM);
s.connect((HOST, int(PORT)))
cmd=''
while (cmd!='quit'):
    try:
        s.settimeout(3.00)
        print (s.recv(1024)+'\n')
    except:
        pass
    s.settimeout(1.00)
    s.sendall(cmd+'\n')
    cmd=raw_input('what now: ')
s.close()
