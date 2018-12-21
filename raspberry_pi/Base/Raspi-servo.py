import wiringpi as wpi
import time

servo_pin  =  13
button_pin = 17


wpi.wiringPiSetupGpio()
wpi.pinMode( servo_pin, 2 )
wpi.pinMode( button_pin, 0 )
wpi.pullUpDnControl( button_pin, 2 )
wpi.pwmSetMode(0)
wpi.pwmSetRange(1024)
wpi.pwmSetClock(375)


while True:
  b = int(input())
  set_degree = b
  pulse = int( 1024 * (1.45 + set_degree * 0.95 /90 ) /20 )
  wpi.pwmWrite( servo_pin, pulse )
  time.sleep(0.1)
