from inside_polygon import Geospacial
from location import LocationObject
import time
from gps import GPS

startTime = time.time()
loopTime = 0.5 # second

DEBUG = True
SAFE_SPOT = {
    "lat": "-28",
    "long": "153"
}

MOTORS = {
    "BLADES": "1234",
    "FRONT_LEFT": "1234",
    "FRONT_RIGHT": "1234",
    "BACK_LEFT": "1234",
    "BACK_RIGHT": "1234"
}

POWER_LEVELS = {
    "50%": "1234"
}

# Outside
# -28.156353, 153.328361
# Inside
# -28.156569, 153.328353
currentLocation = LocationObject(-28.156569, 153.328353)
geospace = Geospacial()
gps = GPS()

def printX(msg):
    if (DEBUG):
        print(msg)

def sendPowerLevelToMotor(motorId, powerLevel):
    try:
        #TODO: Create sending of power level to each motor pin
        raise Exception("Sorry, there is no code here yet.")

        # return True
    except:
        print("Could not send power level of {} to motor with ID: {}".format(powerLevel, motorId))
    
    return False

def start():
    printX("Safe spot set to: {}".format(SAFE_SPOT))
    loop()

def loop():
    while True:
        isInsideGeofence = geospace.determineIfPointLiesWithinPolygon(gps.getPositionData())

        if (isInsideGeofence):
            printX("Inside")
        else:
            printX("Outside")

        # if (sendPowerLevelToMotor(MOTORS["BLADES"], POWER_LEVELS["50%"])):
        #     printX("Blades are on with 50% power.")

        # define all variables
        # find out where we are based on the geofence
        # attempt to go to first point then:
            # turn on blade motors
        # begin navigating the path with blades on
        # if any issues turn off blade motors & navigate back to safe spot
        time.sleep(loopTime - ((time.time() - startTime) % loopTime))

if __name__ == "__main__":
    start()