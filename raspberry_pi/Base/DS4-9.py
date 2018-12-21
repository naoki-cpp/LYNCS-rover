import ctypes
from time import sleep
high =ctypes.CDLL('./bme280.so')
aa = 0

while True:

        a = int(input())
        b = int(input())
        #b = 0
        #a = 100
        aa =  a

        axis1 = int(aa )
        axis5 = 30
        axis2 = 20
        button = b


        print(axis1)
        print(b)

        high.trns(axis1,axis1,axis5,axis5,axis2,axis2,button,button)
        #sleep(0.1)
