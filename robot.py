import pybullet as p
import pybullet_data
import time
import numpy as np

from pybullet_robots.panda.panda_sim_grasp import ll, ul, jr, rp, pandaEndEffectorIndex, pandaNumDofs, jointPositions


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())


p.setGravity(0, 0, -9.81)

planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf")

aabbmin, aabbmax = p.getAABB(tableId)
start_pos = [aabbmin[0]+0.2,0,aabbmax[2]]
start_ori = p.getQuaternionFromEuler([0,0,0])

robotId = p.loadURDF("franka_panda/panda.urdf", start_pos, start_ori, useFixedBase=True)

def set_ja_goal(ja):
    assert len(ja) == 7
    p.setJointMotorControlArray(robotId, jointIndices=[0,1,2,3,4,5,6], controlMode=p.POSITION_CONTROL, targetPositions=ja)

# import ipdb; ipdb.set_trace()

# infos = [p.getJointInfo(robotId,i) for i in range(7)]
# uppers = [info[9] for info in infos]
# lowers = [info[8] for info in infos]
# ranges = [upper - lower for upper, lower in zip(uppers, lowers)]

index=0
for j in range(p.getNumJoints(robotId)):
    p.changeDynamics(robotId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(robotId, j)
    #print("info=",info)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC):
    
        p.resetJointState(robotId, j, jointPositions[index]) 
        index=index+1
    if (jointType == p.JOINT_REVOLUTE):
        p.resetJointState(robotId, j, jointPositions[index]) 
        index=index+1

def get_ik_sol(pos, orient):
    return p.calculateInverseKinematics(
        robotId,
        pandaEndEffectorIndex, pos, down, ll, ul, jr, rp, maxNumIterations=20
        )

# import ipdb; ipdb.set_trace()

# # breakpoint()
# for i in range (100):
#     set_ja_goal([0]*7)
#     p.stepSimulation()
#     time.sleep(1./240.)

# breakpoint()

# down = p.getQuaternionFromEuler([np.pi/2., 0., 0.])
down = [1, 0, 0, 0]

ik = get_ik_sol([-0.2, 0, aabbmax[2]+0.5], down)

for i in range (10000):
    

    set_ja_goal(ik[:7])
    p.stepSimulation()
    time.sleep(1./240.)

