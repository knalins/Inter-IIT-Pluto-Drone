#!/usr/bin/env python3

import struct
import re

def isValidIP(ipaddr):
  if type(ipaddr) is not str:
    raise TypeError("Invalid HOSTIP")

  pattern = r"^((25[0-5]|(2[0-4]|1\d|[1-9]|)\d)\.?\b){4}$"

  isValid = bool(re.search(pattern, ipaddr))

  return isValid

def concatByteArrs(*arrs):
    res = b''
    for i in arrs:
      res += i
    
    return res

def getChecksum(*args):
  byteArr = concatByteArrs(*args)

  checksum = 0
  for byte in byteArr:
    checksum ^= byte

  return struct.pack("<B", checksum)

def createDataPacket(direction, msgLength, payloadType, payloadFormat, payload):
  HEADER = struct.pack("<2c", b'$', b'M')
  DIRECTION = struct.pack("<c", direction.encode('utf-8'))
  MSG_LENGTH = struct.pack("<B", msgLength)
  PAYLOAD_TYPE = struct.pack("<c", payloadType)
  if(payload):
    PAYLOAD = struct.pack(payloadFormat, *payload)
  else:
    PAYLOAD = b""
  CHECKSUM = getChecksum(MSG_LENGTH, PAYLOAD_TYPE, PAYLOAD)

  return concatByteArrs(HEADER, DIRECTION, MSG_LENGTH, PAYLOAD_TYPE, PAYLOAD, CHECKSUM)

if __name__ == '__main__':
  quit()