#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def follow_line(drive_base: DriveBase, color_sensor: ColorSensor, speed: float = 100):
	BLACK = 10
	WHITE = 85
	threshold = (BLACK + WHITE) / 2

	error = color_sensor.reflection() - threshold
	turn_rate = error * 1.5
	drive_base.drive(speed, turn_rate)


def turn_radius(drive_base: DriveBase, radius: float, speed: float = 100):
	turn_rate = speed / radius
	drive_base.drive(speed, turn_rate)

