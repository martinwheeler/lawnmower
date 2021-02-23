# Robot Lawn Mower

_A lawn mower with a brain_

## High Level Overview
- Drives within a geofenced area along a predetermined path that allows the mower to nicely cut the gress
- Turns mower blades off and on depending on the mowers current state and where it is within the geofence/world
- Returns to charge when it has:
    - Run out of battery
    - Finished the path
    - Got stuck or reached a bad status

## Specifics
- Generated path takes into account the width of the blades and makes sure the path slightly overlaps as it mows
- Generated path geo-points are used as a way to store information about previous mows
    - The points will store:
        - Last mow (timestamp)
        - Blade height (millimetres)

### Sensors
- Bump (micro-switches)
- Proximity (sonar)

### Different Statuses
- Heading
- Location
- Traction
- Speed
- Blade Power
- Blade Height
- Battery Percent
- Lean Angle
- Power Consumption
    - Per motor
    - Main board
    - RTK
    - IMU
- Bump Count