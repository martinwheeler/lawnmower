class LocationObject:
    def __init__(self, latitude = 0.0, longitude = 0.0, speed = 0.0, heading = 0.0):
        self.fLongitude = longitude
        self.fLatitude = latitude
        self.speed = speed * 51.444444 # Gives us cm/s from knots
        self.heading = heading