#!/usr/bin/env pybricks-micropython
import sys
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
middle_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, 56, 114)

light_sensor = ColorSensor(Port.S1)
touch_sensor = TouchSensor(Port.S2)
ultrasonic_sensor = UltrasonicSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4)


def main():
	# MAIN LOOP
	while True:
		pass


if __name__ == "__main__":


	main()
	sys.exit(0)