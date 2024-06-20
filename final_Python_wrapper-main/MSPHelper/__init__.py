#!/usr/bin/env python3

# MSPClass defines the structure of given MSP commands
class MSPClass:
  def __init__(self):
    self.prefix__MSP_SET_RAW_RC = ('<', 16, b'\xc8', '<8H')
    self.prefix__MSP_SET_COMMAND = ('<', 2, b'\xd9', '<H')

  # set_raw_rc controlls arming, disarming, roll, pitch, throttle, and yaw
  def SET_RAW_RC(self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4):
    return (*self.prefix__MSP_SET_RAW_RC, [roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4])

  # set_commands controlls takeoff and landing
  def SET_COMMAND(self, command):
    return (*self.prefix__MSP_SET_COMMAND, [command])

MSP = MSPClass()

if __name__ == '__main__':
  quit()
