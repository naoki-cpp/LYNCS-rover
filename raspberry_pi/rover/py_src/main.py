from math import sin
from math import cos
from math import tan
from math import atan2
from math import acos
from math import radians
import rover_module
from bin import lyncs_rover

r = 6378.137
x1 = 139.988909
y1 = 35.685828
while True:
    lyncs_rover.Csearch1()

    gpsdata = rover_module.gps_measurement()
    theta_r = rover_module.convert_lat_long_to_r_theta(gpsdata[0], gpsdata[1],
                                                       x1, y1)

    if theta_r[0] < 1000:
        lyncs_rover.TransferValuesToArduino(theta_r[1] * 1000, 0)
    else:
        lyncs_rover.Csearch2()
