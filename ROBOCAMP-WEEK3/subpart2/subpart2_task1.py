from numpy.core.numeric import Infinity
from numpy.lib.function_base import angle
import pybullet as p
import pybullet_data
import os
import time
from math import atan, cos, sin, acos, asin, atan, pow

file_name = "2R_planar_robot.urdf"
p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 0.05], [0, 0, 0, 0.707])

p.setGravity(0,0,-10)
p.createConstraint(robot, -1, -1, 0,p.JOINT_FIXED, [0,0,0],[0,0,0],[8e-05, 0.02446 ,-0.00146])

'''
Note:
*these are the two end point of the line segment you want to 
 draw,so after finding the line segment you want to trace change 
 the two point below.
*the function addUserDebugLine just draws the line segment joining
point_A and point_B 
'''

point_A = [0,2.04939,0.1]
point_B = [0,-2.04939,0.1]
p.addUserDebugLine(point_A,point_B,[0,1,0],2)

l1 = 1.05
l2 = 1

def Forward_kinematics(angle_1,angle_2):
    '''s space is the y-z pl
    This function should do the necessary trignometric 
    calculations using angle_1,angle_2,l1,l2 and resturn the [0,y,z] as the robot
    workane
    '''
    y=l1*cos(angle_1) + l2*cos(angle_1 + angle_2)
    z=l1*sin(angle_1) + l2*sin(angle_1 + angle_2)

    return [0,y,z]

def Inverse_kinematics(target):
    '''
    This function should do the necessary trignometric 
    calculations using y ,z,l1,l2 and return angle_1 and angle_2 to 
    reach a given target of the form [0,y,z],as the robot 
    works space is th y-z plane.
    '''
    y=target
     
    A=((0.05*0.05)+y*y-l1*l1-l2*l2)/(2*l1*l2)
    B=acos(A)
    a=l1+l2*cos(B)
    b=l2*sin(B)
    C=(2*abs(y)*b + pow((4*y*y*b*b-4*(a*a+b*b)*(y*y-a*a)), 0.5))/(2*(a*a+b*b))
    D=acos(C)
    if(y>0):
        angle_2=-B
        angle_1=-D
    if(y<0):
        angle_2=B
        angle_1=D
    if(y==0):
        angle_2=-3.14
        angle_1=0

    return angle_1,angle_2

'''
Write the equation of the line you are going to follow:-
Example, 
*it might be of the for z = m*y + c, so in the
 while loop below increament y = y + dy and find new z
 and give the target ie [0,y,z] to the Inverse_Kinematics 
 function.
*so trace the line from point_A to point_B and reset position 
 to point_A and continue the same process infinitely.  
z=0.1
-2.04939<y<2.04939
'''
y=2.04939
while(True):

    if(y<=(-2.04939)):
        y=2.04939
        

    angle_1,angle_2  = Inverse_kinematics(y)	

    p.setJointMotorControl2(bodyIndex=robot,
                            jointIndex=0,
                            controlMode =p.POSITION_CONTROL,
                            targetPosition=angle_1,
                            force=500)

    p.setJointMotorControl2(bodyIndex=robot,
                            jointIndex=1,
                            controlMode =p.POSITION_CONTROL,
                            targetPosition=angle_2,
                            force=500)

    p.stepSimulation()
    y=y-0.00001

