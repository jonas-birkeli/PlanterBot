#!/usr/bin/env pybricks-micropython
import sys
import time

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
middle_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, 56, 114)

"""
light_sensor = ColorSensor(Port.S1)
touch_sensor = TouchSensor(Port.S2)
ultrasonic_sensor = UltrasonicSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4)
"""

DRIVESPEED = 100


def turn(radius: float, speed: float = 100):
	turn_rate = speed / radius
	robot.drive(speed, turn_rate)


def end_turn():
	robot.stop()


def plant(plant_speed: float = 100):
	middle_motor.run(plant_speed)


def stop_planting():
	middle_motor.stop()


def main():
	# MAIN LOOP
	plant()
	for n in range(6):  # We want to make the robot drive in a square and drive left and right
		robot.drive(DRIVESPEED, 0)

		time.sleep(10)
		robot.stop()

		if n % 2 == 0:
			turn(1, DRIVESPEED)
		else:
			turn(-1, DRIVESPEED)

		time.sleep(4)
		robot.stop()  # End turn

	stop_planting()


if __name__ == "__main__":
	main()
	sys.exit(0)
