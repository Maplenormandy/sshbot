#!/usr/bin/python
import socket, sys, time, base64, json

HOST=sys.argv[1]
PORT=sys.argv[2]
mapfile=sys.argv[3]
map=open(mapfile, 'r').read()[:-1]
map_cmd='{"token":"666","MAP":"'+map+'"}done\n'
game_cmd='{"token":"666","GAME":"start"}done\n'

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM);
s.connect((HOST, int(PORT)))
s.sendall(map_cmd)
time.sleep(0.5)
s.sendall(game_cmd)
s.close()
