import pybullet as p
import pybullet_data
import time
from six.moves import input

p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
#these are the pre required conditions for the task.
ramp=p.loadURDF("wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])

'''
1.print Number of Joints and joint info and state
2.Get the user input about the control function they 
want to simulate and see.(its upto you, it can be a string / int anything but just leave
a comment with the legend to your menu)
'''
print(p.getNumJoints(husky))
for i in range(p.getNumJoints(husky)):
	print(p.getJointInfo(husky, i))

wheel=[2, 3, 4, 5]

k = input("Enter the control function: ")#enter 1 for torque control and 2 for velocity control

#function to be filled to implement torque control
def Torque_control():
	# find this value to climb the ramp without sliping and topling
    optimal_torque_value = [-250, -250, -250, -250]
    '''
	this function should have the setJointMotorControl in TORQUE_CONTROL configuration
    with force = optimal_force_value
    ''' 
    p.setJointMotorControlArray(husky, wheel, controlMode=p.TORQUE_CONTROL, forces=optimal_torque_value)



#function to be filled to implement velocity control
def Velocity_control():
	# Keep a constant non zero value for maxForce and try getting the velocity that makes it climb the ramp.
	maxForce = [-28, -28, -20, -20]

	# find this value to climb the ramp without sliping
	optimal_velocity_value = [5.8, 5.8, 5, 5]
	'''
	this function should have the setJointMotorControl in VELOCITY_CONTROL configuration
	with targetvelocity = optimal_velocity_value
	'''
	p.setJointMotorControlArray(husky, wheel, controlMode=p.VELOCITY_CONTROL, targetVelocities=optimal_velocity_value, forces=maxForce)

i=0
while (1):
    time.sleep(.01)
    '''
    1.Here call either the Torque_control function or Velocity_control
      function according to the initial user choice and apply the optimal velocity/Torque
      to the motors that you have found by experimentation.
    2.print base state and velocity 100 iteration steps once.
    '''

    if(k=="1"):
	    Torque_control()
    elif(k=="2"):
        Velocity_control()
    else:
        print("Invalid input")
        break
    if(i%100==0):
        print(p.getBasePositionAndOrientation(husky))
        print(p.getBaseVelocity(husky))
    i=i+1

	
    p.stepSimulation()





p.disconnect()
