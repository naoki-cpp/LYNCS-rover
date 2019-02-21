# -*- coding: utf-8 -*-
import pynmea2
import serial
from math import radians
from math import atan2
from math import acos
from math import sin
from math import cos
from math import tan


def lat_long_reader(sentence):
    msg = pynmea2.parse(sentence)
    lat = float(msg.latitude)
    longi = float(msg.longitude)
    return [lat, longi]


def velocity_reader(sentence):
    msg = pynmea2.parse(sentence)
    speed = 0
    course = 0
    if msg.spd_over_grnd != None:
        speed = float(msg.spd_over_grnd)
    if msg.true_course != None:
        course = float(msg.true_course)
    return [speed, course]


def lat_long_measurement():
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    while True:
        sentence = s.readline().decode('utf-8')  # GPSデーターを読み、文字列に変換する
        if sentence[0] == '$' and ('GGA' in sentence or 'RMC' in sentence
                                   or 'GLL' in sentence):
            lat_and_long = lat_long_reader(sentence)
            break
    return [lat_and_long[0], lat_and_long[1]]

def velocity_measurement():
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    while True:
        sentence = s.readline().decode('utf-8')  # GPSデーターを読み、文字列に変換する
        if sentence[0] == '$' and ('RMC' in sentence):
            gps_data = velocity_reader(sentence)
            break
    return [gps_data[0], gps_data[1]]

r = 6378.137  # km


def convert_lat_long_to_r_theta(lat0, long0, lat1, long1):
    y0 = radians(lat0)
    x0 = radians(long0)
    y1 = radians(lat1)
    x1 = radians(long1)
    deltax = x1 - x0

    theta = atan2(sin(deltax), (cos(y0) * tan(y1) - sin(y0) * cos(deltax)))
    distance = r * acos(sin(y0) * sin(y1) + cos(y0) * cos(y1) * cos(deltax))
    return [distance, theta]


def r_theta_to_goal(goal_lat, goal_long):
    current_coordinate = lat_long_measurement()
    return convert_lat_long_to_r_theta(
        current_coordinate[0], current_coordinate[1], goal_lat, goal_long)
