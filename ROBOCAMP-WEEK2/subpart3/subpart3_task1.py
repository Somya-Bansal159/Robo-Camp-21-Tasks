import pybullet as p
import pybullet_data
import time
from six.moves import input

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
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

    p.stepSimulation()


p.getContactPoints(car)

p.disconnect()


