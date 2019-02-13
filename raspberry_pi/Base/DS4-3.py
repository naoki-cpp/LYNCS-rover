import ctypes
from time import sleep
test=ctypes.CDLL('./sendSPI.so')
aa = 0

while True:

        a = int(input())
        b = int(input())
        #a = 100
        #b = 20
        aa =  a

        axis1 = int(aa )
        axis2 = b
        axis5 = 10

        print(axis1)
        print(b)


        test.trns(axis1,axis2,axis5,2)
        #sleep(0.1)
