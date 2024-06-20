#!/usr/bin/env python3

import struct
import re

# check for IP if its valid or not using regexp  
def isValidIP(ipaddr):
  if type(ipaddr) is not str:
    raise TypeError("Invalid HOSTIP")

  pattern = r"^((25[0-5]|(2[0-4]|1\d|[1-9]|)\d)\.?\b){4}$"

  isValid = bool(re.search(pattern, ipaddr))

  return isValid

# make a byte array as per the requirnment of multiwii protocol and telnet communication
def concatByteArrs(*arrs):
    res = b''
    for i in arrs:
      res += i
    
    return res

# perform bitwise xor to find the checksum for MSP commands
def getChecksum(*args):
  byteArr = concatByteArrs(*args)

  checksum = 0
  for byte in byteArr:
    checksum ^= byte
  # struct.pack return a object from the parameter in given format
  return struct.pack("<B", checksum)

# create and return data packet (byte array) based on multiwii serial protocol (header '$M', direction '>/<', msgLength, payloadType, payload, checksum)
def createDataPacket(direction, msgLength, payloadType, payloadFormat, payload):
  HEADER = struct.pack("<2c", b'$', b'M')
  DIRECTION = struct.pack("<c", direction.encode('utf-8'))
  MSG_LENGTH = struct.pack("<B", msgLength)
  PAYLOAD_TYPE = struct.pack("<c", payloadType)
  PAYLOAD = struct.pack(payloadFormat, *payload)
  CHECKSUM = getChecksum(MSG_LENGTH, PAYLOAD_TYPE, PAYLOAD)

  return concatByteArrs(HEADER, DIRECTION, MSG_LENGTH, PAYLOAD_TYPE, PAYLOAD, CHECKSUM)

if __name__ == '__main__':
  quit()
