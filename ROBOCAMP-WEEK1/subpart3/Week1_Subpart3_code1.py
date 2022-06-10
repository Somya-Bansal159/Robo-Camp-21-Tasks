import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
planeId = p.loadURDF("plane.urdf")

cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId1 = p.loadURDF("sample.urdf",[2,2,1], cubeStartOrientation)
boxId2 = p.loadURDF("dabba.urdf",[0,0,1], cubeStartOrientation)

i=0
while(True):
    j=1
    t=i/240.0
    g=t
    while(g>9.8):
       g=t-9.8*j
       j=j+1
    p.setGravity(g/1.414, g/1.414, 0)
    i=i+1
    p.stepSimulation()
    time.sleep(1./240.)
    print(g)#magnitude of gravity is printed on the terminal


