def parse_map(mapString):
    parts = mapString.split(":")
    i = 0
    parseGridSize(parts[i++])
    startPose = parsePose(parts[i++])
    parts = parts[i:]
    for x in len(parts)
			addWall(parseWall(parts[x]))


def parseGridSize(part):
    


def parsePose(poseString):
    parts = poseString.split(",")


def addWall(wallString):
    parts = wallString.split(",")
    
