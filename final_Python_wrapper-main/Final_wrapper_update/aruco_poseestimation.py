import Aruco_class
import cv2
import math
from PID import PID
from Pluto import Pluto
import time
import keyboard

# Offsets introduced to mke the ground under the camera global (0,0,0)
XOFFSET = 0.8
YOFFSET = 0.36
ZOFFSET = 3.01

# # Values
# XOFFSET = 1.2
# YOFFSET = 0.1
# ZOFFSET = 3.3


zcor=0
# Aruco tag definition and object call
aruco = Aruco_class.Aruco("DICT_5X5_100")

# To deploy the webcam
aruco.begin_camera(len=1024, wid=1024, camera_no=0)
time.sleep(2)
drone = Pluto('192.168.4.1', 23)
drone.arm()
drone.take_off()
drone.move_up(.7)



# Target distance from Camera
targetdist_z = 1.5
targetdist_y = 0
targetdist_x = 0


#PID loop for height
pidheight_z = PID(P=300, I=0, D=100)

pidheight_z.SetPoint = targetdist_z

# pidheight_y= PID(P=0, I=100, D=0)
# pidheight_y.SetPoint = targetdist_y

# pidheight_x = PID(P=0, I=100, D=0)
# pidheight_x.SetPoint = targetdist_x


r,p=0,0
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
        # pidheight_y.update(ycor)
        # pidheight_x.update(xcor)
        print(f"   Zcor: {zcor},    Target: {targetdist_z},   error: {round(pidheight_z.output)}")

    con=60
    if(keyboard.is_pressed('p')):
        pidheight_z.Kp+=10  
    if(keyboard.is_pressed('l')):
        pidheight_z.Kp-=10
    if(keyboard.is_pressed('o')):
        pidheight_z.Ki+=10
    if(keyboard.is_pressed('k')):
        pidheight_z.Ki-=10
    if(keyboard.is_pressed('i')):
        pidheight_z.Kd+=10
    if(keyboard.is_pressed('j')):
        pidheight_z.Kd-=10

    
    if(keyboard.is_pressed('w')):
        p+=con
        t+=100
    if(keyboard.is_pressed('a')):
        r-=con
        t+=100
    if(keyboard.is_pressed('s')):
        p-=con
        t+=100
    if(keyboard.is_pressed('d')):
        r+=con
        t+=100
    if(keyboard.is_pressed('r')):
        p=0
        r=0

    # if(keyboard.is_pressed('q')):
        # drone.land()

    # t=round(pidheight_z.output)+abs(p)+abs(r)
    t=round(pidheight_z.output)

    if(t>600):
        t=600
    if(t<-600):
        t=-600
    if(r>500):
        r=500
    if(r<-500):
        r=-500
    if(p>500):
        p=500
    if(p<-500):
        p=-500  

    # print(f'r:{1500+r}  p:{1500+p}  t:{1500+t}  pid:{pidheight_z.Kp} {pidheight_z.Ki} {pidheight_z.Kd}')
    drone.hover2(r,p,t) #call with single argument
    

    # # Grid to locate the aruco in the camera setup
    # start_point = (0, 0)
    # end_point = (1280, 960)
    # start_point2 = (1040, 780)
    # end_point2 = (240, 180)
    # color = (0, 0, 0)
    # thickness = 1
    # output = cv2.rectangle(output, start_point, end_point, color, thickness)
    # output = cv2.rectangle(output, start_point2, end_point2, color, thickness*3)
    # for i in range(240, 1040, 50) :
    #     output = cv2.line(output, (i, 180), (i, 780), color, thickness)
    # for i in range(180, 780, 50) :
    #     output = cv2.line(output, (1040, i), (240, i), color, thickness)
    # cv2.circle(output, (640,480), 5, color, thickness)
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
