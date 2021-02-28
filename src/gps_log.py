import time
import serial

ser = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)
counter=0
text_file = open('/home/lawn/code/lawnmower/gps_log.txt', 'w')


while 1:
    x=ser.readline().decode('utf-8')
    print(x, file=text_file)