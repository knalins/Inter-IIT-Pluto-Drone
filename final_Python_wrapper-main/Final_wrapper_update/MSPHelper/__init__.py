#!/usr/bin/env python3

class MSPClass:
  def __init__(self):
    self.prefix__MSP_SET_RAW_RC = ('<', 16, b'\xc8', '<8H')
    self.prefix__MSP_SET_COMMAND = ('<', 2, b'\xd9', '<H')
    self.prefix__MSP_RAW_IMU = ('<', 0, b'\x66', '')

  def SET_RAW_RC(self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4):
    return (*self.prefix__MSP_SET_RAW_RC, [roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4])

  def SET_COMMAND(self, command):
    return (*self.prefix__MSP_SET_COMMAND, [command])
  
  def RAW_IMU(self):
    return (*self.prefix__MSP_RAW_IMU, [])

MSP = MSPClass()

if __name__ == '__main__':
  quit()