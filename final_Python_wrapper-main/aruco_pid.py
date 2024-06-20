import Aruco_class
import cv2
from PID import PID
from Pluto import Pluto
import time

# Offsets introduced to mke the ground under the camera global (0,0,0)
XOFFSET = 0.018
YOFFSET = -0.085
ZOFFSET = 1.20

zcor=0
# Aruco tag definition and object call
aruco = Aruco_class.Aruco("DICT_5X5_100")

# To deploy the webcam
aruco.begin_camera(len=1024, wid=1024, camera_no=0)
time.sleep(4)
drone = Pluto('192.168.4.1', 23)
drone.arm()
drone.move_up(5)

# Target height
targetdist_z = 0.80


#PID loop for height
pidheight_z = PID(P=1000, I=10, D=1)
pidheight_z.SetPoint = targetdist_z

while aruco.cap.isOpened():

    ret, img = aruco.cap.read()

    output, rot, trans = aruco.pose_estimation(img)

    if (rot is not None) and (trans is not None):
        # To show Camera Feed Comment out if not required


        # Print Rotational and Translational vectors
        # print(f"rvec{rot}, tvec{trans}")
        xcor = round(trans[0][0][0] + XOFFSET, 2)
        ycor = round(trans[0][0][1] + YOFFSET, 2)
        zcor = round(ZOFFSET - trans[0][0][2], 2)
        # print(f"Xcor: {xcor}, Ycor: {ycor}, Zcor: {zcor}")

        # print((rotation_matrix_to_attitude_angles(rmat)))
        pidheight_z.update(zcor)

        # print(f"Zcor: {zcor}, Target: {targetdist_z}, error: {round(pidheight_z.output)}")

    #run the hover PID command in the loop
    drone.hover(0,1500+round(pidheight_z.output))


    #To show the image on the screen
    cv2.imshow('Estimated Pose', output)

    # Exit Conditions
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Close the Camera
drone.land()

drone.disarm()
drone.disconnect()
aruco.end_camera()
