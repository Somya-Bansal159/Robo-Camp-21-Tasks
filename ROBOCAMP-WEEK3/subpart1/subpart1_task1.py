import pybullet as p
import pybullet_data
import time
from six.moves import input
from math import cos, sin
import cv2
import numpy as np

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
p.loadURDF("cube.urdf", [3,0,0.5])
p.loadURDF("cube.urdf", [0,3,0.5])
carpos = [0, 0, 0.1]

car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
targetVel = 10  #rad/s
maxForce = 100 #Newton
#p.applyExternalForce(car,3,[100,0,0],)
x=0
while (1):
    maxForce=100
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        base_position=p.getBasePositionAndOrientation(car)[0]
        base_orientation=p.getEulerFromQuaternion(p.getBasePositionAndOrientation(car)[1])
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

        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = 3+x
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
           
            p.stepSimulation()

        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -3-x
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            maxForce = 200+x
            for joint in [3, 5]:
                p.setJointMotorControl2(car, joint, p.TORQUE_CONTROL,force = maxForce)
            for joint in [2, 4]:
                p.setJointMotorControl2(car, joint, p.TORQUE_CONTROL,force = -maxForce)

            p.stepSimulation()
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            maxForce = 200+x
            for joint in [3, 5]:
                p.setJointMotorControl2(car, joint, p.TORQUE_CONTROL,force = -maxForce)
            for joint in [2, 4]:
                p.setJointMotorControl2(car, joint, p.TORQUE_CONTROL,force = maxForce)

            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity = targetVel, force = maxForce)
 
            p.stepSimulation()

        if (k == 114L and (v & p.KEY_IS_DOWN)):
            maxForce = 215+x
            for joint in [3, 5]:
                p.setJointMotorControl2(car, joint, p.TORQUE_CONTROL,force = maxForce)
            for joint in [2, 4]:
                p.setJointMotorControl2(car, joint, p.TORQUE_CONTROL,force = -maxForce)

            p.stepSimulation()

    	if (k == 114L and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity = targetVel, force = maxForce)
 
            p.stepSimulation()

        if (k == 97L and (v & p.KEY_WAS_RELEASED)):
            x=x+1

        if (k == 99L and (v & p.KEY_IS_DOWN)):
            image = p.getCameraImage(width,
                                     height,
                                     view_matrix,
                                     projection_matrix,
                                     shadow=True,
                                     renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgb_opengl = (np.reshape(image[2], (height, width, 4)) * 1. / 255.)
            
            cv2.imshow("win", rgb_opengl)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            p.stepSimulation()
            

    p.stepSimulation()


p.getContactPoints(car)

p.disconnect()


