import sys
#import and init pygame
import pygame

def parse_map(mapString):
    parts = mapString.split(":")
    i = 0
    gridSize = float(parseGridSize(parts[i++]))
    startPose = parsePose(parts[i++])
    parts = parts[i:]
    for x in len(parts)
			addWall(parseWall(parts[x]))


def parseGridSize(part):
    return 


def parsePose(poseString):
    parts = poseString.split(",")
    
    return 


def addWall(wallString):
    parts = wallString.split(",")


if __name__ == "__main__":
    pygame.init() 

    #create the screen
    window = pygame.display.set_mode((500, 500)) 

    pygame.draw.line(window, (255, 255, 255), (0, 0), (500, 500))

    #draw it to the screen
    pygame.display.flip() 

    pygame.image.save(window, "map/map.png")

    #input handling (somewhat boilerplate code):
    while True: 
       for event in pygame.event.get(): 
          if event.type == pygame.QUIT: 
              sys.exit(0)
