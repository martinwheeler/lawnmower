from time import sleep
from BMI160_i2c import Driver

print('Trying to initialize the sensor...')
sensor = Driver(0x69) # Default address is 0x69
print('Initialization done')


try:
    while True:
        data = sensor.getMotion6()
        rotation = sensor.getRotation()
        # fetch all gyro and acclerometer values
        print({
            'gx': data[0],
            'gy': data[1],
            'gz': data[2],
            'ax': data[3],
            'ay': data[4],
            'az': data[5]
        })

        # print('x', data[0])

        # temperature = sensor.getTemperature()

        # print(temperature)
        # print('y', data[1])
        # print('z', data[2])

        sleep(1)
except KeyboardInterrupt:
    pass