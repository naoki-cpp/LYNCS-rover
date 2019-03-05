from bin import lyncs_rover
import time


cs = lyncs_rover.arduino_control()
if cs.Init() == -1:
    print('error')
    exit()
while True: 
    cs.Csearch1()