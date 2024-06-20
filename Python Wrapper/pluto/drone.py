import struct
import telnetlib
import numpy as np
from threading import Thread
from .enforce_types import enforce_types

@enforce_types
class Drone:

    def __init__(self, DroneIP="192.168.4.1", DronePort="23"):
        self.DRONEIP = DroneIP
        self.DRONEPORT = DronePort
        try:
            self.tn = telnetlib.Telnet(self.DRONEIP, self.DRONEPORT)
            print("pluto connected")
        except:
            print("Error While Connecting to Pluto")
        self.RC_ROLL, self.RC_PITCH, self.RC_THROTTLE, self.RC_YAW = 1500, 1500, 1500, 1500

    def disconnect(self):
        self.tn.close()
        print("Server connection closed")

    def message(self,data, typeOfMsg):
        HEADER = [b'$', b'M']
        DIRECTION = {"IN": b'<', "OUT": b'>'}
        MSP_MSG_PARSE = '<3c2B%iHB'
        # typeofmsg 217 set or 200 rc
        lenOfData = len(data)
        msg = HEADER + [DIRECTION["IN"]] + [lenOfData * 2] + [typeOfMsg] + data
        msg = struct.pack(MSP_MSG_PARSE[:-1] % lenOfData, *msg)
        
        checksum = 0
        for i in msg[3:]:
            checksum ^= i
        msg += bytes([checksum])
        return msg
    
    def arming(self, arm: bool):
        """
        Parses the arm and disarm commands.

        Parameters
        ----------
        arm : bool
            True for arm and False for disarm.
        Returns
        -------
        parsed : bytes
            The parsed data to be sent to the drone.
        """

        RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW, RC_AUX1, RC_AUX2, RC_AUX3, RC_AUX4 = 1500, 1500, 1000, 1500, 1200, 1500, 1500, 1500
        data = [RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW, RC_AUX1, RC_AUX2, RC_AUX3, RC_AUX4]
        if arm:
            data[-1] = 1500
        else:
            data[-1] = 901

        parsed = self.message(data, 200)
        return parsed
    
    def takeoff(self):
        data=[1]
        parsed=self.message(data, 217)
        self.sendData(parsed, "Takeoff")

    def land(self):
        data=[2]
        parsed=self.message(data, 217)
        self.sendData(parsed, "land")

    def backflip(self):
        data=[3]
        parsed=self.message(data, 217)
        self.sendData(parsed, "backflip")

    def trim(self, roll, pitch, throttle, yaw):
        self.RC_ROLL += roll
        self.RC_PITCH += pitch
        self.RC_THROTTLE += throttle
        self.RC_YAW += yaw

    def steer_cmd(self, direction:str, magnitude:int=100):
        """
        Parses the steer commands.

        Parameters
        ----------
        direction : str
            Valid inputs - "forward", "backward", "left", "right", "up", "down".
        magnitude : int
            Magnitude over which the drone steers, -600<magnitude<600

        Returns
        -------
        parsed : bytes
            The parsed data to be sent to the drone.
        """

        center = np.array([self.RC_ROLL, self.RC_PITCH, self.RC_THROTTLE, self.RC_YAW])

        RC_AUX1, RC_AUX2, RC_AUX3, RC_AUX4 = 1500, 1500, 1500, 1500
    
        if magnitude + 1500 > 2100:
            print("Clipping magnitude to 2100")
            magnitude = 600
        if magnitude + 1500 < 900:
            print("Clipping magnitude to 900")
            magnitude = -600

        change = {
            "forward": np.array([0, magnitude, 0, 0]),
            "backward": np.array([0, -magnitude, 0, 0]),      
            "left": np.array([-magnitude, 0, 0, 0]),      
            "right": np.array([magnitude, 0, 0, 0]),
            "up": np.array([0, 0, magnitude, 0]),
            "down": np.array([0, 0, -magnitude, 0]),
            "clck": np.array([0, 0, 0, magnitude]),
            "anticlck": np.array([0, 0, 0, -magnitude]),
        }
    
        RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW,  = center + change[direction]
        data = [RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW, RC_AUX1, RC_AUX2, RC_AUX3, RC_AUX4]
        parsed = self.message(data, 200)
        return parsed

    def set_steer_data(self, magnitude):
        """
        Parses the steer commands.

        Parameters
        ----------
        magnitude : array-like
            Magnitude of roll, pitch, throttle, and yaw commands -600<magnitude<600

        Returns
        -------
        parsed : bytes
            The parsed data to be sent to the drone.
        """

        center = np.array([self.RC_ROLL, self.RC_PITCH, self.RC_THROTTLE, self.RC_YAW])

        RC_AUX1, RC_AUX2, RC_AUX3, RC_AUX4 = 1500, 1500, 1500, 1500
        for i in range(4):
            if magnitude[i] + 1500 > 2100:
                print("Clipping magnitude to 2100")
                magnitude[i] = 600
            if magnitude[i] + 1500 < 900:
                print("Clipping magnitude to 900")
                magnitude[i] = -600
    
        RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW,  = center + np.array(magnitude)
        data = [RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW, RC_AUX1, RC_AUX2, RC_AUX3, RC_AUX4]
        parsed = self.message(data, 200)
        return parsed

    def arm(self):
        self.sendData(self.arming(True), "ARM")
    
    def disarm(self):
        self.sendData(self.arming(False), "DISARM")

    def steer(self, direction:str, magnitude:int=100):
        self.sendData(self.steer_cmd(direction, magnitude), f"STEER {direction}")
        
    def set_steer(self, magnitude):
        if len(magnitude) != 4:
            print("Invalid legth of message array. format: [roll, pitch, throttle, yaw]")
        self.sendData(self.set_steer_data(magnitude), f"Sending {magnitude}")

    def sendData(self, data, cmd):
        try:
            print(data)
            self.tn.write(data)
        except:
            print("Error While sending {} Data".format(cmd))
