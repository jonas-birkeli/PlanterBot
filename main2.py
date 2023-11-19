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
medium_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, 56, 114)
gyro = GyroSensor(Port.S4)
touch = TouchSensor(Port.S2)


DRIVESPEED = 100
LINES = 4


class Robot:
	def __init__(self, drive_base: DriveBase, middle_motor: Motor, gyro_sensor: GyroSensor) -> None:
		self.drive_base = drive_base
		self.middle_motor = middle_motor
		self.gyro_sensor = gyro_sensor

		self.gyro_sensor.reset_angle(0)

		if gyro_sensor.angle() == 0:
			ev3.speaker.beep(duration=100, frequency=1000)
			# Simple check if angle has been reset

		self.drive_base.reset()
		self.drive_base.settings(
			straight_speed=DRIVESPEED, straight_acceleration=DRIVESPEED * 2,
			turn_rate=DRIVESPEED * 0.8, turn_acceleration=DRIVESPEED * 0.8
		)
		self.drive_base.stop()

	def start(self) -> None:
		self.middle_motor.run(100)  # Start planting

		for n in range(LINES):  # We want to make the robot drive in a square by driving up and down
			self.__drive_and_turn(1000, 5, (n % 2)*2 - 1)
			# (n % 2)*2 - 1 is a fancy way of saying -1 if n is odd, 1 if n is even

		self.middle_motor.stop()  # Stop planting

	def __drive_and_turn(self, distance: float, radius: float = 1, direction: int = 1) -> None:
		"""
		Drives the robot in a straight line for a given distance, then turns it in a circle of a given radius for 180 degrees

		:param distance: Distance of the robot in a straight line
		:param radius: Radius of the circle the robot will turn in
		:param direction: Direction of the circle the robot will turn in
		:return: None
		"""
		self.drive_base.straight(distance)

		while True:
			self.drive_base.drive(DRIVESPEED / 2, (90/radius)*direction)
			if self.gyro_sensor.angle() >= 180 or self.gyro_sensor.angle() <= 1:
				self.drive_base.stop()
				break


if __name__ == "__main__":
	robot = Robot(robot, medium_motor, gyro)
	robot.start()
