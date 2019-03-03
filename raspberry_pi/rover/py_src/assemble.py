from bin import lyncs_rover
import rover_module as gps
from rover_module import height
from time import sleep

goal_lat, goal_log = [35.555744, 139.654071]

cs = lyncs_rover.arduino_control()
if cs.Init() == -1:
    print('error')

count = 0
while True:
    count += 1
    judge_data0 = height.readData()
    if count % 10 == 1:
        cs.LogOutput('phase1, height::' + str(judge_data0))
    if judge_data0 > height.max_high:
        break

while True:
    count += 1
    judge_data = height.readData()
    if count % 10 == 1:
        cs.LogOutput('phase2, height::' + str(judge_data))

    if judge_data < height.low_high and height.math.fabs(height.given_data -
                                                         judge_data) < 0.8:
        break
    height.given_data = judge_data

cs.LogOutput('landed.')
cs.Transfer(0,6)
sleep(5)

length, theta = [0,0]
while True:
    list_dis_thet = gps.r_theta_to_goal(goal_lat, goal_log)
    if list_dis_thet is not None:
        length, theta = list_dis_thet
        break

while True:

    list_dis_thet = gps.r_theta_to_goal(goal_lat, goal_log)
    if list_dis_thet is not None:
        length, theta = list_dis_thet

    coord = gps.lat_long_measurement()
    if coord is not None:
        cs.LogOutput('lat::' + str(coord[0]) + ', long::' + str(coord[1]))
    length_theta = gps.r_theta_to_goal(goal_lat, goal_log)
    if length_theta is not None:
        cs.LogOutput('dist::' + str(length) + ', angle::' + str(theta))
    for i in range(25):
        judge = cs.Csearch1()
        if length * 1000 < 5 and judge == 1:
            cs.Csearch2()
    # f r_theata[0]*1000 < 20:
    #    cs.Csearch2()
    # else:
    if length * 1000 >= 5:
        cs.Transfer(int(theta * 1000), 5)
