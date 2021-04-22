import serial
from location import LocationObject

SERIAL_PORT = "/dev/ttyTHS1"
running = True
gps = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)
defaultLocation = LocationObject(0.0, 0.0)

class GPS:
    # In the NMEA message, the position gets transmitted as:
    # DDMM.MMMMM, where DD denotes the degrees and MM.MMMMM denotes
    # the minutes. However, I want to convert this format to the following:
    # DD.MMMM. This method converts a transmitted string to the desired format
    def formatDegreesMinutes(self, coordinates, digits):
        
        parts = coordinates.split(".")

        if (len(parts) != 2):
            return coordinates

        if (digits > 3 or digits < 2):
            return coordinates
        
        left = parts[0]
        right = parts[1]
        degrees = str(left[:digits])
        minutes = str(right[:10])

        return degrees + "." + minutes

    # This method reads the data from the serial port, the GPS dongle is attached to,
    # and then parses the NMEA messages it transmits.
    # gps is the serial port, that's used to communicate with the GPS adapter
    def getPositionData(self):
        data = gps.readline().decode('utf-8')
        message = data[0:6]
        if (message == "$GNRMC"):
            # GPRMC = Recommended minimum specific GPS/Transit data
            # Reading the GPS fix data is an alternative approach that also works
            parts = data.split(",")
            if parts[2] == 'V':
                # V = Warning, most likely, there are no satellites in view...
                print("GPS receiver warning")
                return defaultLocation
            else:
                # Get the position data that was transmitted with the GPRMC message
                # In this example, I'm only interested in the longitude and latitude
                # for other values, that can be read, refer to: http://aprs.gids.nl/nmea/#rmc
                latitude = self.formatDegreesMinutes(parts[3], 2)
                longitude = self.formatDegreesMinutes(parts[5], 3)            
                return LocationObject(float(latitude), float(longitude))
        else:
            # Handle other NMEA messages and unsupported strings
            return defaultLocation