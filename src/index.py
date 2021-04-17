import geopy
from geopy.distance import geodesic

SAFE_SPOT = {
    "lat": "",
    "long": ""
}

bladesOn = False
atFirstPoint = False
driving = False
readyToNavigatePath = False
endLoop = False

def goTo(point):
    global driving
    global atFirstPoint
    print(f'Going to {point}')
    driving = True
    atFirstPoint = True

def getPath(pointToReturn):
    print('Generating path to follow')
    return geopy.Point(-28.1567392, 153.3280381)

def getDistanceToPoint(point):
    print(f'Getting distance to point {point}')

def blades(on):
    global bladesOn
    print(f'Turning blades {on}')
    if on:
        bladesOn = True
    else:
        bladesOn = False

def navigatePath():
    global endLoop
    print('Navigating path and mowing')

    # Look at current point travelling to
        # Get within a certain range of that point
        # Then mark it as reached and start looking at the next point

    endLoop = True

def start():
    global firstPoint
    firstPoint = getPath(0)
    loop()

def detectIssues():

    # check blade motors
    # check wheel motors
    # check position based on path
    # check battery levels
    print('Checking for any problems.')
    return False

def loop():
    global atFirstPoint
    global readyToNavigatePath

    detectIssues()
        
    # begin navigating the path with blades on
    if readyToNavigatePath:
        navigatePath()
    else:
        if not(atFirstPoint):
            goTo(firstPoint)
        else:
            if not(bladesOn):
                blades(True)
                readyToNavigatePath = True

    # if any issues turn off blade motors & navigate back to safe spot
    if not(endLoop):
        loop()
    

if __name__ == "__main__":
    start()