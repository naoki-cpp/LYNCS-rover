from math import sin
from math import cos
from math import tan
from math import atan2
from math import acos
from math import radians
import RoverGPS
from bin import lyncs_rover

r = 6378.137
x1 = 139.988909
y1 = 35.685828
x1 = radians(x1)
y1 = radians(y1)
while True:
    lyncs_rover.Csearch1()

    gpsdata = RoverGPS.gps_measurement()

    x2 = radians(gpsdata[1])
    y2 = radians(gpsdata[0])

    deltax = x2 - x1
    ans = atan2(sin(deltax), (cos(y1) * tan(y2) - sin(y1) * cos(deltax)))
    distance = r * acos(sin(y1) * sin(y2) + cos(y1) * cos(y2) * cos(deltax))
    if distance < 1000:
        lyncs_rover.TransferValuesToArduino(ans * 1000, 0)
    else:
        lyncs_rover.Csearch2()
