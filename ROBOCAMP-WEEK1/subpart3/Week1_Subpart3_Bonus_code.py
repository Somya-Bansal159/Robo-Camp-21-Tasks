import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

list=[]
def fib(i):
    if(i==1):
        return 0
    elif(i==2):
        return 1
    else:
        return fib(i-1)+fib(i-2)
i=1
while(True):
    p.stepSimulation()
    if (i==1):
        i = i +1
        continue
    for j in range (fib(i-1)):
        p.resetBasePositionAndOrientation(list[j], [j, 0, 5], cubeStartOrientation)
    for j in range(fib(i-1),fib(i)):
        list.append(p.loadURDF("sphere.urdf",[j,0,5], cubeStartOrientation))
    cubePos, cubeOrn = p.getBasePositionAndOrientation(list[0])
    print("fuck")
    for z in range(10000):
        p.stepSimulation()
        print(z)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(list[0])
        if (cubePos[2]<0.2):
            break
        time.sleep(1./240.)
    i=i+1
    print("off")
