#!/usr/bin/env python
import sys
import pygame
from beginner_tutorials.srv import *
import rospy

size_factor = 0
silo = []
reactor = []
opponent = []

def parsePose(poseString):
    parts = poseString.split(",")
    x = float(parts[0])*size_factor
    y = float(parts[1])*size_factor
    theta = float(parts[2])
    return (x, y, theta)


def parseWall(wallString):
    parts = wallString.split(",")
    x1 = float(parts[0])*size_factor
    y1 = float(parts[1])*size_factor
    x2 = float(parts[2])*size_factor
    y2 = float(parts[3])*size_factor
    color = parts[4]
    return ((x1, y1), (x2, y2), color)


def handle_locator(req):
    if req.label == "silo":
        return silo[0]


if __name__ == "__main__":
    mapString = "22.0:4.5,5.5,3.14159:"+\
                "0,0,0,1,N:0,1,1,2,O:1,2,1,3,O:1,3,0,4,O:0,4,0,6,S:0,6,3,6,N:3,6,4,6,R:4,6,7,6,N:7,6,7,1,N:7,1,6,0,N:"+\
                "6,0,5,0,R:5,0,3,0,N:3,0,2,1,N:2,1,1,0,N:1,0,0,0,R:" +\
                "4,0,4,5,N:4,5,5,5,N:" +\
                "4,1,3,2,N:3,2,2,2,N:2,2,2,3,N:2,3,4,3,N:" +\
                "5,1,5,3,N:5,3,6,3,N:6,3,6,2,N:6,2,5,1,N:"
    
    parts = mapString.split(":")
    i = 0
    gridSize = float(parts[i])
    size_factor = gridSize*2.54
    i += 1
    startPose = parsePose(parts[i])
    i += 1
    parts = parts[i:]
    
    max_x = 0
    max_y = 0
    walls = []

    for x in range(len(parts)):
        if parts[x] != "":
            wall = parseWall(parts[x])
            walls.append(wall)
            #print str(wall[0]) + " " + str(wall[1])
            #pygame.draw.line(window, (255, 255, 255), wall[0], wall[1])
            if wall[0][0]>max_x:
                max_x = int(round(wall[0][0]))
            if wall[0][1]>max_y:
                max_y = int(round(wall[0][1]))
            if wall[1][0]>max_x:
                max_x = int(round(wall[1][0]))
            if wall[1][1]>max_y:
                max_y = int(round(wall[1][1]))

            if wall[2] == "O":
                opponent.append((wall[0][0]+wall[1][0])/2.0) 
                opponent.append((wall[0][1]+wall[1][1])/2.0)
            elif wall[2] == "R":
                reactor.append(((wall[0][0]+wall[1][0])/2.0, (wall[0][1]+wall[1][1])/2.0))
            elif wall[2] == "S":
                silo.append(((wall[0][0]+wall[1][0])/2.0, (wall[0][1]+wall[1][1])/2.0))
            
    #pygame.init() 
    window = pygame.display.set_mode((max_x+1, max_y+1)) 
    
    for wall in walls:
        pygame.draw.line(window, (255, 255, 255), wall[0], wall[1])

    window = pygame.transform.flip(window, False, True)

    pygame.image.save(window, "map/map.png")


    f = open('map/map.yaml','w')
    f.write('image: map.png\n')
    f.write('resolution: 0.01\n')
    #f.write('origin: [0.0, 0.0, 0.0]\n')
    f.write('origin: ['+str(-startPose[0])+','+str(-startPose[1])+','+str(startPose[2])+']\n')
    f.write('occupied_thresh: 0.65\n')
    f.write('free_thresh: 0.196\n')
    f.write('negate: 0')
    f.close()

    rospy.init_node('locator_server')
    s = rospy.Service('locator', locator, handle_locator)
    rospy.spin()

#    #draw it to the screen
#    pygame.display.flip() 
#    #input handling (somewhat boilerplate code):
#    while True: 
#       for event in pygame.event.get(): 
#          if event.type == pygame.QUIT: 
#              sys.exit(0)

    

