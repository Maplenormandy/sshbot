#!/usr/bin/python
import socket, sys, time, base64, json

HOST=sys.argv[1]
PORT=sys.argv[2]
commands=open('emulator_commands.txt', 'r');
image=base64.b64encode(open('../pics/tux.png', 'r').read())
image_cmd='{"token":"1221","IMAGE_DATA":"'+str(image)+'"}done'

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM);
s.connect((HOST, int(PORT)))
s.settimeout(1.00);
cmd=''
i=0
while (cmd!='quit'):
    #try:
    #    print (s.recv(1024)+'\n')
    #except:
    #    pass
    cmd=commands.readline()
    if (cmd==''):
        commands.seek(0)
        cmd=commands.readline()
    if (i==5):
        s.sendall(image_cmd+'\n')
        i=0
        time.sleep(0.2)
        continue
    s.sendall(cmd)
    i+=1
    time.sleep(0.25)
s.close()
