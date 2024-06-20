import Aruco_class
import cv2
import math
from PID import PID
from Pluto import Pluto
import time

# Offsets introduced to mke the ground under the camera global (0,0,0)
XOFFSET = 0.75
YOFFSET = 0.4
ZOFFSET = 3

zcor=0
# Aruco tag definition and object call
aruco = Aruco_class.Aruco("DICT_5X5_100")

# To deploy the webcam
aruco.begin_camera(len=1024, wid=1024, camera_no=0)
drone = Pluto('192.168.4.1', 23)
drone.arm()
drone.take_off()
# drone.move_up(0.2)


# Target distance from Camera
targetdist_z = 1.5
targetdist_y = 0
targetdist_x= 0


#PID loop for height
pidheight_z = PID(P=200, I=2, D=20)
pidheight_z.SetPoint = targetdist_z

pidheight_y= PID(P=50, I=0, D=5)
pidheight_y.SetPoint = targetdist_y

pidheight_x = PID(P=50, I=0, D=5)
pidheight_x.SetPoint = targetdist_x



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

        print(f"Xcor: {xcor}, Ycor: {ycor}, Zcor: {zcor}")


        # rmat = cv2.Rodrigues(rot)[0]
        # print((rotation_matrix_to_attitude_angles(rmat)))
        pidheight_z.update(zcor)
        pidheight_y.update(ycor)
        pidheight_x.update(xcor)
        print(f"Zcor: {zcor}, Target: {targetdist_z}, error: {round(pidheight_z.output)}")
        # print(f"ycor: {ycor}, Target: {targetdist_y}, error: {round(pidheight_y.output)}")
        # print(f"xcor: {xcor}, Target: {targetdist_x}, error: {round(pidheight_x.output)}")

    drone.hover2(round(-1*pidheight_x.output),round(pidheight_y.output),round(pidheight_z.output)) #call with single argument
    # drone.hover2(round(pidheight_z.output))

    # Exit Conditions
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        drone.land()
        break

# Close the Camera
drone.land()

drone.disarm()
drone.disconnect()
aruco.end_camera()
