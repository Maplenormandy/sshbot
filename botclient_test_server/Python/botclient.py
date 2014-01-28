#!/usr/bin/python
import sys
import json
import pkg_resources

from multiprocessing import Process, Queue, Value
from twisted.internet import protocol, reactor
from twisted.protocols import basic

#for websockets
from autobahn.twisted.websocket import WebSocketServerFactory, WebSocketServerProtocol, listenWS

import botprotocol, websocketrelayprotocol

filename='userdb.json'   #json formatted as {token: user, ...}

class WebSocketBotRelayFactory(WebSocketServerFactory):
    protocol=websocketrelayprotocol.BotRelayProtocol

    def __init__(self, url, debug=False, debugCodepaths=False):
        WebSocketServerFactory.__init__(self, url)
        #print url

'''class BotRelayFactory(protocol.ServerFactory):
    protocol=relayprotocol.BotRelayProtocol

    def __init__(self):
        self.connected=False
        self.targetuser=None

    def isconnected(self):
        return self.connected

    def connect(self):
        self.connected=True

    def disconnect(self):
        self.connected=False

    def settargetuser(self, user):
        self.targetuser=user

    def gettargetuser(self):
        return self.targetuser
'''
class BotFactory(protocol.ServerFactory):
    protocol = botprotocol.BotProtocol

    def __init__(self):
        self.connections=0
        self.db=json.loads(open(filename, 'r').read())
        #print self.db
        self.clients=[]

    def addConnection(self, client):
        self.connections+=1
        if not client in self.clients:
            self.clients.append(client)
        print str(client) + ' has connected\n'

    def removeConnection(self, client):
        self.connections-=1
        if client in self.clients:
            self.clients.remove(client)
        print str(client) + ' has disconnected\n'

    def check_token(self, token):
        if (token in self.db):
            return self.db[token]
        return False

    def broadcast(self, message):
        for c in self.clients:
            c.transport.write(message)

reactor.listenTCP(6667, BotFactory())
listenWS(WebSocketBotRelayFactory("ws://localhost:9000", True, True))
#reactor.listenTCP(6668, BotRelayFactory())
#reactorc.listenTCP(6669, BotRelayFactory())
reactor.run()
#reactorc.run()
