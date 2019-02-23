import rover_module as gps
from time import sleep

cs = lyncs_rover.arduino_control()
if cs.Init() == -1:
    print('error')
    exit()
while True:
    length, theta = gps.r_theta_to_goal(35.555225, 139.654664)
    for i in range(25):
        judge1=cs.Csearch1()
        if judge1==1 and length*1000<20:
            cs.Csearch2()
    #if r_theata[0]*1000 < 20:
    #    cs.Csearch2()
    #else:
    if length*1000=>20:
        cs.Transfer(int(theta*1000), 5)
