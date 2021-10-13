import pybullet as p
import pybullet_data
import time
import numpy as np

from pybullet_robots.panda.panda_sim_grasp import rp, pandaEndEffectorIndex, pandaNumDofs, jointPositions

from ompl_demo import StateChecker, create_plan


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


INITIAL_POSITION = jointPositions
INITIAL_POSITION = [
    1.3785442497282303,
    0.5454914443278179,
    0.7205662023986104,
    -0.6335492641212528,
    -0.5172454347100123,
    2.596939552397514,
    2.3206010363085565,
    0.02,
    0.02,
]

index=0
for j in range(p.getNumJoints(robotId)):
    p.changeDynamics(robotId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(robotId, j)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC):
        p.resetJointState(robotId, j, INITIAL_POSITION[index]) 
        index=index+1
    if (jointType == p.JOINT_REVOLUTE):
        p.resetJointState(robotId, j, INITIAL_POSITION[index]) 
        index=index+1


# for i in range(1000):
#     p.stepSimulation()
#     time.sleep(1./240.)

# index=0
# for j in range(p.getNumJoints(robotId)):
#     info = p.getJointInfo(robotId, j)
#     jointName = info[1]
#     jointType = info[2]
#     if (jointType == p.JOINT_PRISMATIC):
#         print(p.getJointState(robotId, j)[0])
#         index=index+1
#     if (jointType == p.JOINT_REVOLUTE):
#         print(p.getJointState(robotId, j)[0])
#         index=index+1

def get_ik_sol(pos, orient, ll, ul, jr):
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

def get_joint_bounds(robotID, simID):
    '''
    Get joint bounds.
    By default, read from pybullet
    '''
    joint_bounds = []
    for i, joint_id in enumerate(range(7)):
        joint_info = p.getJointInfo(robotID, joint_id, physicsClientId=simID)
        low = joint_info[8] # low bounds
        high = joint_info[9] # high bounds
        if low < high:
            joint_bounds.append([low, high])
    print("Joint bounds: {}".format(joint_bounds))
    return joint_bounds

bounds = get_joint_bounds(robotId, physicsClient)
ll = [l[0] for l in bounds]
ul = [l[1] for l in bounds]
jr = [l[1] - l[0] for l in bounds]


ik = get_ik_sol([-0.2, 0, aabbmax[2]+0.1], down, ll, ul, jr)


checker = StateChecker()
# breakpoint()
if not checker.is_state_valid(INITIAL_POSITION[:7]):
    raise ValueError("BLAHBLAHBLAH")
# print(ul)
# print(ll)

# for i, joint_id in enumerate(self.joint_idx):
#         joint_info = p.getJointInfo(self.id, joint_id)
#         low = joint_info[8] # low bounds
#         high = joint_info[9] # high bounds
#         if low < high:
#             self.joint_bounds.append([low, high])
#     print("Joint bounds: {}".format(self.joint_bounds))
#     return self.joint_bounds

plan = create_plan(INITIAL_POSITION[:7], ik[:7], joint_limits=bounds, checker=checker)


# breakpoint()

for i in range (len(plan)):
    

    # set_ja_goal(ik[:7])
    set_ja_goal(plan[i])
    p.stepSimulation()
    time.sleep(1./240.)

for i in range(10000):

    p.stepSimulation()
    time.sleep(1./240.)
