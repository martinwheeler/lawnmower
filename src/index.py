import numpy
import geopy
from geopy.distance import geodesic
import pathPlanning
from pathPlanning import planning
import threading
import random
from gps import GPS
from inside_polygon import Geospacial
from location import LocationObject
import time
from datetime import datetime
import struct
import serial

SAFE_SPOT = {
    "lat": "",
    "long": ""
}

# https://www.jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/

REQUIRED_DISTANCE_TO_GPS_POINT = 1 # metres

previousLocation = 0
recording = False
recordTimestamp = 0
recordFile = 0
bladesOn = False
reachedStartingPoint = False
driving = False
endLoop = False
px = []
py = []

boundaryX = [-28.1567392,-28.1568149,-28.1567688,-28.1567392,-28.1567357,-28.1567365,-28.1567424,-28.1566336,-28.1565603,-28.1565130,-28.1565106,-28.1565036,-28.1564882,-28.1565213,-28.1565650,-28.1566253,-28.1567392]
boundaryY = [153.3280381,153.3281870,153.3282420,153.3282997,153.3283386,153.3284806,153.3285516,153.3285744,153.3285852,153.3285074,153.3284323,153.3283491,153.3282392,153.3281895,153.3281185,153.3280809,153.3280381]
pathResolution = 0.00001
locationIndex = 0
pathIndex = 0

gps = GPS()
geospace = Geospacial()

motor1 = serial.Serial(
    port="/dev/ttyTHS2",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)
motor2 = serial.Serial(
    port="/dev/ttyTHS3",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

def changeLocationIndex():
    threading.Timer(1.0, changeLocationIndex).start()
    global locationIndex
    locationIndex = random.randint(0, 20)

# TODO: Pull from GPS module
def getGPSLocation():
    global locationIndex
    currenctLocation = gps.getPositionData()
    return geopy.Point(currenctLocation.fLatitude, currenctLocation.fLongitude)
    #return geopy.Point(px[locationIndex], py[locationIndex])

def sendMotorSignal(steer = 0, speed = 20, motor):
    start=0xABCD
    checksum = start ^ steer ^ speed
    
    # https://www.journaldev.com/17401/python-struct-pack-unpack
    dataToSend = struct.pack('HhhH', start, steer, speed, checksum)

    motor.write(dataToSend)

def turnLeft():
    sendMotorSignal(0, -20, motor1)
    sendMotorSignal(0, 20, motor2)

def turnRight():
    sendMotorSignal(0, 20, motor1)
    sendMotorSignal(0, -20, motor2)

def forward():
    sendMotorSignal(0, 20, motor1)
    sendMotorSignal(0, 20, motor2)

def backward():
    sendMotorSignal(0, -20, motor1)
    sendMotorSignal(0, -20, motor2)

def goTo(point):
    return
    # Turn the machine to face the direction of the next GPS point
        # We can use the GPS heading measurement
    # Move forward until reached the desired point

    # I guess if it moves off track of the point we will have to re-adjust and make it face the point again
        # Can look at and see if the current heading matches or is close to an ideal heading
            # Once the heading angle is too far off we can turn again to focus on the correct heading

    #print(f'Going to {point}')

def getPath(pointToReturn):
    global px, py
    return geopy.Point(px[pointToReturn], py[pointToReturn])

def measure_gps_distance(start, end):
    return numpy.rint(geopy.distance.geodesic(start, end).km / 1000)

def getDistanceToPoint(point):
    distance = measure_gps_distance(getGPSLocation(), point)
    print(f'Distance to point {point} is {distance} metres')
    return distance

def blades(turnOn):
    global bladesOn
    if turnOn != bladesOn:
        print(f'Turning blades {"on" if turnOn else "off"}')

    if turnOn:
        bladesOn = True
    else:
        bladesOn = False

def navigatePath():
    global endLoop, pathIndex

    nextGPSPoint = getPath(pathIndex)
    goTo(nextGPSPoint)

    if getDistanceToPoint(nextGPSPoint) <= REQUIRED_DISTANCE_TO_GPS_POINT:
        pathIndex = pathIndex + 1
        print(f'Navigating to next point {getPath(pathIndex)}')

def detectIssues():

    # check blade motors
    # check wheel motors
    # check position based on path
    # check battery levels
    # print('Checking for any problems.')
    return False

def handleMowing():
    global pathIndex

    if pathIndex > 0 and pathIndex < len(px) and not(bladesOn):
        blades(True)
    
    if pathIndex == 0 or pathIndex == len(px):
        blades(False)

def startRecording():
    global recordTimestamp, recordFile, recording
    recordTimestamp = datetime.now()
    recordFile = open(f'/home/lawn/code/lawnmower/logs/gps_log_{recordTimestamp}.txt', 'w')
    recording = True    

def stopRecording():
    global recording, recordFile
    print('Stopping recording')
    recordFile = 0
    recording = False

def saveToFile(message, filePath):
    print(message, file=filePath, flush=True)

def recordFence():
    global recordFile, previousLocation
    try:
        currentLocation = getGPSLocation()

        if currentLocation != previousLocation:
            saveToFile(f'{datetime.now()} {currentLocation}', recordFile)
            previousLocation = currentLocation
    except:
        print('No file ready')

def start():
    global firstPoint, px, py
    px, py = planning(boundaryX, boundaryY, pathResolution)
    firstPoint = getPath(pathIndex)

    firstValue = 12
    secondValue = 10

    print(firstValue^secondValue)

    # if not(recording):
    #     startRecording()

    changeLocationIndex()
    loop()

def loop():
    global reachedStartingPoint, pathIndex, recording

    if recording:
        recordFence()
    else:
        detectIssues()
        navigatePath()
        handleMowing()
    
    threading.Timer(0.01, loop).start()

if __name__ == "__main__":
    start()