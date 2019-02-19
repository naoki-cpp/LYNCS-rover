from bin import lyncs_rover
import rover_module as gps
from time import sleep

goal_lat = 0
goal_long =0
cs = lyncs_rover.arduino_control()
cs.Init()
while True:
    cs.Csearch1()
    r_theata=gps.r_theta_to_goal(goal_lat, goal_long)
    if r_theata[0] < 20:
        cs.Csearch2()
    else:
        lyncs_rover.TransferValuesToArduino((int)(r_theata[0]/1000), 0)
    sleep(1)
