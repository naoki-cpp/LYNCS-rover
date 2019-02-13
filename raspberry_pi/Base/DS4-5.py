import ctypes
from time import sleep
test=ctypes.CDLL('./sendSPI2.so')
aa = 0

while True:

        a = int(input())
        b = int(input())
        #b = 0
        #a = 100
        aa =  a

        axis1 = int(aa )
        button = b
        axis5 = 30

        print(axis1)
        print(b)

        test.trns(axis1,axis1,axis5,axis5,20,20,10,10,button,button)
        #sleep(0.1)
