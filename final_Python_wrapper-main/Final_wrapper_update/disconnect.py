#!/usr/bin/env python3
from Pluto import Pluto

drone = Pluto("192.168.4.1", 23)
drone.arm()
drone.disarm()
drone.disconnect()
