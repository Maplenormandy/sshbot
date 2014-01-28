#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('nav')
import socket 
import Queue
import pygame
import rospy
import math
import tf
from nav.srv import *
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

class locator():
    def __init__(self, mapString):
        self.parseMap(mapString)
        self.srv = rospy.Service('locator', Locator, self.handle_locator)

    def parseMap(self, mapString):
        self.silo = []
        self.reactor = []
        self.opponent = []
        self.max_x = 0
        self.max_y = 0
        walls = []
        i = 0
        self.robotRadius = 11 #in inches
        self.resolution = 0.05 #in meters per pixel

        parts = mapString.split(":")
        gridSize = float(parts[i])
        self.size_factor = gridSize #inches
        i += 1
        self.startPose = self.parsePose(parts[i])
        i += 1
        parts = parts[i:]

        for x in range(len(parts)):
            if parts[x] != "":
                wall = self.parseWall(parts[x]) #parsed in inches
                walls.append(wall)
                #print str(wall[0]) + " " + str(wall[1])
                if wall[0][0]>self.max_x:
                    self.max_x = wall[0][0]
                if wall[0][1]>self.max_y:
                    self.max_y = wall[0][1]
                if wall[1][0]>self.max_x:
                    self.max_x = wall[1][0]
                if wall[1][1]>self.max_y:
                    self.max_y = wall[1][1]

                if wall[2] == "O":
                    xpos = (wall[0][0]+wall[1][0])/2.0
                    ypos = (wall[0][1]+wall[1][1])/2.0
                    ang = math.atan2(-(wall[0][0]-wall[1][0]), (wall[0][1]-wall[1][1]))
                    self.opponent.append([xpos, ypos, ang])
                elif wall[2] == "R":
                    xpos = (wall[0][0]+wall[1][0])/2.0
                    ypos = (wall[0][1]+wall[1][1])/2.0
                    ang = math.atan2(-(wall[0][0]-wall[1][0]), (wall[0][1]-wall[1][1]))
                    self.reactor.append([xpos, ypos, ang])
                elif wall[2] == "S":
                    xpos = (wall[0][0]+wall[1][0])/2.0
                    ypos = (wall[0][1]+wall[1][1])/2.0
                    ang = math.atan2(-(wall[0][0]-wall[1][0]), (wall[0][1]-wall[1][1]))
                    self.silo.append([xpos, ypos, ang])

        self.max_x_actual = int(round(self.max_x*0.0254/self.resolution))+1 #pixels
        self.max_y_actual = int(round(self.max_y*0.0254/self.resolution))+1
        window = pygame.Surface((self.max_x_actual, self.max_y_actual))

        for wall in walls:
            x0 = int(round(wall[0][0]*0.0254/self.resolution))
            y0 = int(round(wall[0][1]*0.0254/self.resolution))
            x1 = int(round(wall[1][0]*0.0254/self.resolution))
            y1 = int(round(wall[1][1]*0.0254/self.resolution))
            pygame.draw.line(window, (255, 0, 0), (x0,y0), (x1,y1))

        self.flood_fill(window, self.startPose[0], self.startPose[1])
        pxarr = pygame.PixelArray(window)
        pxarr.replace((255, 0, 0), (0, 0, 0))
        window = pxarr.make_surface()

        self.correctPoses(window)
        #window = self.displayPoses(window)
        window = pygame.transform.flip(window, False, True)

        pygame.image.save(window, "catkin_ws/src/nav/map/map.png")

        f = open('catkin_ws/src/nav/map/map.yaml','w')
        f.write('image: map.png\n')
        f.write('resolution: '+str(self.resolution)+'\n')
        #f.write('origin: [0.0, 0.0, 0.0]\n')

        #x0 = self.startPose[0]*self.resolution
        #y0 = self.startPose[1]*self.resolution
        #dtheta = self.startPose[2]

        #theta = math.atan2(y0, x0)+dtheta
        #r = (x0**2+y0**2)**(.5)

        #x1 = r*math.cos(theta)
        #y1 = r*math.sin(theta)

        #f.write('origin: ['+str(-x1)+','+str(-y1)+','+str(dtheta)+']\n')
        #f.write('origin: ['+str(-x0)+','+str(-y0)+',0]\n')
        f.write('origin: [0,0,0]\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')
        f.write('negate: 0')
        f.close()



    def flood_fill(self, image, x, y):#x, y values are in inches
        x = int(round(x*0.0254/self.resolution))
        y = int(round(y*0.0254/self.resolution))
        queue = Queue.Queue()
        queue.put((x, y))
        while not queue.empty():
            #print queue.qsize()
            curr = queue.get()
            (x, y) = curr
            if (x<self.max_x_actual and x>=0 and y<self.max_y_actual and y>=0 and image.get_at((x, y)) == (0, 0, 0)):
                image.set_at((x, y), (255,255,255))
                queue.put((x+1,y))
                queue.put((x,y+1))
                queue.put((x,y-1))
                queue.put((x-1,y))

    def correctPoses(self, image): #corrects coordinates to meters
        for i in self.opponent:
            x = int(round((i[0] + self.robotRadius*math.cos(i[2]))*0.0254/self.resolution))
            y = int(round((i[1] + self.robotRadius*math.sin(i[2]))*0.0254/self.resolution))
            if not ((x > self.max_x_actual or x<0) or (y > self.max_y_actual or y<0) or (image.get_at((x, y)) == (0, 0, 0))):
                i[2] = normalize(i[2]+math.pi)
        for i in self.reactor:
            x = int(round((i[0] + self.robotRadius*math.cos(i[2]))*0.0254/self.resolution))
            y = int(round((i[1] + self.robotRadius*math.sin(i[2]))*0.0254/self.resolution))
            if not ((x > self.max_x_actual or x<0) or (y > self.max_y_actual or y<0) or (image.get_at((x, y)) == (0, 0, 0))):
                i[2] = normalize(i[2]+math.pi)
        for i in self.silo:
            x = int(round((i[0] + self.robotRadius*math.cos(i[2]))*0.0254/self.resolution))
            y = int(round((i[1] + self.robotRadius*math.sin(i[2]))*0.0254/self.resolution))
            if not ((x > self.max_x_actual or x<0) or (y > self.max_y_actual or y<0) or (image.get_at((x, y)) == (0, 0, 0))):
                i[2] = normalize(i[2]+math.pi)

        for i in self.opponent:
            i[0] = (i[0] + self.robotRadius*math.cos(normalize(i[2]+math.pi)))*0.0254
            i[1] = (i[1] + self.robotRadius*math.sin(normalize(i[2]+math.pi)))*0.0254
        for i in self.reactor:
            i[0] = (i[0] + self.robotRadius*math.cos(normalize(i[2]+math.pi)))*0.0254
            i[1] = (i[1] + self.robotRadius*math.sin(normalize(i[2]+math.pi)))*0.0254
        for i in self.silo:
            i[0] = (i[0] + self.robotRadius*math.cos(normalize(i[2]+math.pi)))*0.0254
            i[1] = (i[1] + self.robotRadius*math.sin(normalize(i[2]+math.pi)))*0.0254


    def displayPoses(self, image):
        print (int(round(self.startPose[0]*0.0254/self.resolution)), int(round(self.startPose[1]*0.0254/self.resolution)))
        pygame.draw.circle(image, (0,255,0), (int(round(self.startPose[0]*0.0254/self.resolution)), int(round(self.startPose[1]*0.0254/self.resolution))), 5)
        for i in self.opponent:
            x = int(round(i[0]/self.resolution))
            y = int(round(i[1]/self.resolution))
            pygame.draw.circle(image, (0,0,255), (x, y), 5)
        for i in self.reactor:
            x = int(round(i[0]/self.resolution))
            y = int(round(i[1]/self.resolution))
            pygame.draw.circle(image, (0,0,255), (x, y), 5)
        for i in self.silo:
            x = int(round(i[0]/self.resolution))
            y = int(round(i[1]/self.resolution))
            pygame.draw.circle(image, (0,0,255), (x, y), 5)
        pygame.draw.circle(image, (255,0,0), (0,0), 10)
        return image

    def parsePose(self, poseString):
        parts = poseString.split(",")
        x = float(parts[0])*self.size_factor
        y = float(parts[1])*self.size_factor
        theta = float(parts[2])
        return (x, y, theta)

    def parseWall(self, wallString):
        parts = wallString.split(",")
        x1 = float(parts[0])*self.size_factor
        y1 = float(parts[1])*self.size_factor
        x2 = float(parts[2])*self.size_factor
        y2 = float(parts[3])*self.size_factor
        color = parts[4]
        return ((x1, y1), (x2, y2), color)



    def handle_locator(self, req):
        resp = LocatorResponse()
        resp.header = Header()
        resp.reactor1 = Pose()
        resp.reactor2 = Pose()
        resp.reactor3 = Pose()
        resp.silo = Pose()
        resp.opponent = Pose()

        resp.header.stamp = rospy.Time.now()
        resp.header.frame_id = "map"

        resp.reactor1.position.x = self.reactor[0][0]
        resp.reactor1.position.y = self.reactor[0][1]
        reactor1quat = tf.transformations.quaternion_from_euler(0, 0, self.reactor[0][2])
        resp.reactor1.orientation.x = reactor1quat[0]
        resp.reactor1.orientation.y = reactor1quat[1]
        resp.reactor1.orientation.z = reactor1quat[2]
        resp.reactor1.orientation.w = reactor1quat[3]

        resp.reactor2.position.x = self.reactor[1][0]
        resp.reactor2.position.y = self.reactor[1][1]
        reactor2quat = tf.transformations.quaternion_from_euler(0, 0, self.reactor[1][2])
        resp.reactor2.orientation.x = reactor2quat[0]
        resp.reactor2.orientation.y = reactor2quat[1]
        resp.reactor2.orientation.z = reactor2quat[2]
        resp.reactor2.orientation.w = reactor2quat[3]

        resp.reactor3.position.x = self.reactor[2][0]
        resp.reactor3.position.y = self.reactor[2][1]
        reactor3quat = tf.transformations.quaternion_from_euler(0, 0, self.reactor[2][2])
        resp.reactor3.orientation.x = reactor3quat[0]
        resp.reactor3.orientation.y = reactor3quat[1]
        resp.reactor3.orientation.z = reactor3quat[2]
        resp.reactor3.orientation.w = reactor3quat[3]

        resp.silo.position.x = self.silo[0][0]
        resp.silo.position.y = self.silo[0][1]
        siloquat = tf.transformations.quaternion_from_euler(0, 0, self.silo[0][2])
        resp.silo.orientation.x = siloquat[0]
        resp.silo.orientation.y = siloquat[1]
        resp.silo.orientation.z = siloquat[2]
        resp.silo.orientation.w = siloquat[3]

        resp.opponent.position.x = self.opponent[0][0]
        resp.opponent.position.y = self.opponent[0][1]
        opponentquat = tf.transformations.quaternion_from_euler(0, 0, self.opponent[0][2])
        resp.opponent.orientation.x = opponentquat[0]
        resp.opponent.orientation.y = opponentquat[1]
        resp.opponent.orientation.z = opponentquat[2]
        resp.opponent.orientation.w = opponentquat[3]
        return resp

def normalize(num):
    while num>math.pi:
        num -= 2.0*math.pi
    while num<-math.pi:
        num += 2.0*math.pi
    return num


if __name__ == "__main__":
    s = socket.socket()         # Create a socket object
    host = socket.gethostname() # Get local machine name
    #host = "18.150.7.174"      # The actual server for competition
    port = 6667 
    s.connect((host, port))
    mapString = s.recv(1024).strip()
    print mapString
    s.close                     # Close the socket when done

    rospy.init_node('locator_server')
    loc = locator(mapString)
    rospy.spin()


