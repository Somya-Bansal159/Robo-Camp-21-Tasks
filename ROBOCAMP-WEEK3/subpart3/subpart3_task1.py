import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from math import cos, sin
p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

huskypos = [0, 0, 0.1]
target_block=p.loadURDF("block0.urdf",-3,0,0)

husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
p.createConstraint(husky, -1, -1, -1, p.JOINT_POINT2POINT, [0, 1, 0], [0, 0,0 ], [0, 0,0])


maxForce = 200 #Newton.m
#camera should be facing in the direction of the car



def position():
    base_position=p.getBasePositionAndOrientation(husky)[0]
    base_orientation=p.getEulerFromQuaternion(p.getBasePositionAndOrientation(husky)[1])
    husky_length=0.2
    focal_length=10
    camera_position=[base_position[0]+husky_length*cos(base_orientation[2]), base_position[1]+husky_length*sin(base_orientation[2]), base_position[2]+0.5]
    focus=[base_position[0]+focal_length*cos(base_orientation[2]), base_position[1]+focal_length*sin(base_orientation[2]), base_position[2]+0.5]
    view_matrix=p.computeViewMatrix(camera_position, focus, [0, 0,1])



    width = 1024
    height = 1024
    
    fov = 120
    aspect = width / height
    near = 0.01
    far = 20
    
    
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    image = p.getCameraImage(width,
                             height,
                             view_matrix,
                             projection_matrix,
                             shadow=True,
                             renderer=p.ER_BULLET_HARDWARE_OPENGL)
    
    
    rgb_opengl = image[2]

    hsv_lower = np.array([50,200,0])
    hsv_upper = np.array([100, 255, 100])
    rgb_opengl = np.asarray(rgb_opengl, dtype=np.uint8)
    rgb_opengl = cv2.cvtColor(rgb_opengl, cv2.COLOR_RGB2HSV)
    rgb_opengl = cv2.inRange(rgb_opengl, hsv_lower, hsv_upper)

    contours, _ = cv2.findContours(rgb_opengl, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    if(len(contours)>0) and (cv2.contourArea(contours[0])>50):
        cv2.drawContours(rgb_opengl, contours=contours, contourIdx=0, color=[0,0,255], thickness=2)

        M = cv2.moments(contours[0])
        cx = int(M['m10'] / M['m00'])
        # cv2.imshow("win", rgb_opengl)
        # cv2.waitKey(1000)
        # cv2.destroyAllWindows()
        return cx
    else:
        turn(500)
        # cv2.imshow("win", rgb_opengl)
        # cv2.waitKey(1000)
        # cv2.destroyAllWindows()
        return position()

    




def turn(speed):
    baseSpeed = 100
    targetVel_R = baseSpeed + speed
    targetVel_L = baseSpeed - speed
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL, targetVelocity =targetVel_R,force = maxForce)
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity =targetVel_L,force = maxForce)
    p.stepSimulation()
'''
tune the kp and kd from experiments, 
a hint set kd = 10*kp
'''
Kp=1
Kd=10
PID_CONTROL=False


first = True
while (1):
    keys = p.getKeyboardEvents()
    if (PID_CONTROL):

        error = position() - 511
        print(error, position())
        if ( first == True):
            speed_correction = Kp*error
        else:
            speed_correction = Kp*error + Kd*(error - last_error)*240
            first = False
        turn(speed_correction)
        last_error = error #initialize accordingly
        
    for k, v in keys.items():

            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN) and PID_CONTROL==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint+1, p.VELOCITY_CONTROL,targetVelocity =-1*targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)and PID_CONTROL==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL,targetVelocity =-1* targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

                p.stepSimulation()
            if (k == ord('c') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-on")
                PID_CONTROL = True
            if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-off ,back to manual")
                PID_CONTROL = False
    time.sleep(1./240.)
p.disconnect()