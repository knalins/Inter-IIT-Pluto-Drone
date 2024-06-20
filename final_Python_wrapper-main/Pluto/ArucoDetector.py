import numpy as np
import cv2

ARUCO_DICT = {"DICT_5X5_100": cv2.aruco.DICT_5X5_100}

class Aruco:
    def __init__(self, aruco_type):
        self.aruco_type = aruco_type

        self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

        self.arucoParams = cv2.aruco.DetectorParameters_create()

        self.intrinsic_camera = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
        self.distortion = np.array((-0.43948, 0.18514, 0, 0))

    def start_camera(self, len, wid, camera_no = 0):
        self.cap = cv2.VideoCapture(camera_no)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, len)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, wid)

    def end_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def pose_estimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_type])
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters, )

        if len(corners) > 0:
            for i in range(0, len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.intrinsic_camera, self.distortion)

                cv2.aruco.drawDetectedMarkers(frame, corners)

            return (frame, rvec, tvec)

        return None