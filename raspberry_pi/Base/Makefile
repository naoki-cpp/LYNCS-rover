.PHONY: clean

bme280: bme280.c bme280.h
	gcc -g -Wall -Wextra -pedantic -std=c11 -D_DEFAULT_SOURCE -D_BSD_SOURCE -shared -o bme280.so bme280.c -lwiringPi -lm 

clean:
	rm bme280
