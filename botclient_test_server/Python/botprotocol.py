from twisted.internet import protocol, reactor
from twisted.protocols import basic
import json, handler, re, time

#this needs to be improved so that the token must not be checked every command. Probably the best improvement is to save the session
#in the factory by checking the token once

done_regex=re.compile('done')
ping_regex=re.compile('ping\n')

class BotProtocol(basic.LineReceiver):
    MAX_LENGTH=90000
    def connectionMade(self):
        self.factory.addConnection(self)
        self.user=False
        self.tmpline=''
        #First Test Map:
        #mapString = "22.00:4.50,5.50,3.14:0.00,1.00,0.00,2.00,R:0.00,2.00,1.00,3.00,N:1.00,3.00,0.00,3.00,N:0.00,3.00,0.00,4.00,N:0.00,4.00,1.00,5.00,N:1.00,5.00,3.00,5.00,N:3.00,5.00,3.00,4.00,N:3.00,4.00,4.00,5.00,N:4.00,5.00,5.00,5.00,N:5.00,5.00,5.00,3.00,N:5.00,3.00,4.00,2.00,N:4.00,2.00,4.00,1.00,N:4.00,1.00,3.00,0.00,N:3.00,0.00,2.00,0.00,N:2.00,0.00,2.00,1.00,N:2.00,1.00,0.00,1.00,N:"
        #Mock 2:
        #mapString = "22.0:4.5,5.5,3.14159:0,0,0,1,N:0,1,1,2,O:1,2,1,3,O:1,3,0,4,O:0,4,0,6,S:0,6,3,6,N:3,6,4,6,R:4,6,7,6,N:7,6,7,1,N:7,1,6,0,N:6,0,5,0,R:5,0,3,0,N:3,0,2,1,N:2,1,1,0,N:1,0,0,0,R:4,0,4,5,N:4,5,5,5,N:4,1,3,2,N:3,2,2,2,N:2,2,2,3,N:2,3,4,3,N:5,1,5,3,N:5,3,6,3,N:6,3,6,2,N:6,2,5,1,N:"
        
        #IDC Map - Hexagon: top of hexagon is virtual silo
        #mapString = "22.0:1.0,1.0,0:0,0,0,3,N:0,3,1,4,N:1,4,2,4,S:2,4,3,3,N:3,3,3,2,R:3,3,3,2,R:3,3,3,2,R:3,2,3,0,N:3,0,0,0,O:" 
        #IDC Map with middle wall awkwardly inbetween
        #mapString = "22.0:1.0,1.0,0:1,0,0,1,R:0,1,0,3,N:0,3,1,4,N:1,4,2,4,S:2,4,3,3,N:3,3,3,2,R:3,3,3,2,R:3,2,3,0,N:3,0,1,0,O:"

        #Next House v1: 
        #mapString = "22.0:1,1,3.14159:0,0,0,3,N:0,3,0,4,S:0,4,2,4,O:2,4,2,3,N:2,3,3,2,N:3,2,3,1,R:3,1,3,0,N:3,0,0,0,N:"
        #Next House v2: 
        #mapString = "22.0:1.0,1.0,0.0:0,0,0,3,N:0,3,0,4,S:0,4,1,4,O:1,4,2,4,R:2,4,2,3,N:2,3,3,2,N:3,2,3,1,R:3,1,3,0,N:3,0,2,0,N:2,0,1,0,R:1,0,0,0,N"
        #self.transport.write("connected\n")
        #time.sleep(2)
        mapString = "22.00:4.00,6.00,-2.36:1.00,3.00,1.00,4.00,N:1.00,4.00,0.00,5.00,N:0.00,5.00,0.00,6.00,N:0.00,6.00,1.00,6.00,N:1.00,6.00,1.00,7.00,N:1.00,7.00,1.00,8.00,N:1.00,8.00,2.00,8.00,R:2.00,8.00,4.00,8.00,S:4.00,8.00,5.00,7.00,N:5.00,7.00,6.00,6.00,N:6.00,6.00,5.00,5.00,N:5.00,5.00,6.00,4.00,N:6.00,4.00,5.00,3.00,R:5.00,3.00,4.00,3.00,N:4.00,3.00,4.00,4.00,N:4.00,4.00,4.00,5.00,N:4.00,5.00,3.00,4.00,N:3.00,4.00,3.00,3.00,N:3.00,3.00,2.00,3.00,N:2.00,3.00,1.00,3.00,R:"
        self.transport.write(mapString)
        #time.sleep(3)
        #self.transport.write("start\n")
        #print "started"


    def connectionLost(self, reason):
        self.factory.removeConnection(self)

    def dataReceived(self, line):
        print line+'\n'
	result=ping_regex.match(line)
        if (result!=None):
            self.transport.write("pong\r\n")
            return
        line=line[:-1]
        self.tmpline+=line
        print line[-4:]
        result=done_regex.match(line[-4:])
        if (result!=None):
            print 'ok parsing line\n'
            self.tmpline=self.tmpline[:-(len(result.group(0)))]
            self.parseline()
            self.tmpline=''
        else:
            return
    def parseline(self):
        print 'input: ' + str(self.tmpline) + '\n'
        try:
            data=json.loads(self.tmpline)
        except:
            print 'failed decoding line\n'
            return
        print 'decoded json: ' + str(data) + '\n'
        self.user=self.factory.check_token(data['token'])
        if (self.user==False):
            print 'invalid user\n'
            return
        elif (self.user=='admin'):
            print 'admin is here Oo\n'
            del data['token']
            if ('MSG' in data):
                commands=data.items()
                dispatch=[]
                for i in commands:
                    dispatch+=[self.user]+[x for x in i]
                handler.addtoqueue(dispatch)
            else:
                self.factory.broadcast(json.dumps(data))
        else:
            print 'valid user\n'
            del data['token']
            commands=data.items()
            dispatch=[]
            for i in commands:
                dispatch+=[self.user]+[x for x in i]
            print 'whats going in the queue: ' + str(dispatch)+'\n'
            handler.addtoqueue(dispatch)
