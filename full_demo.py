import pybullet as p
import pybullet_data
import time
import numpy as np

from pybullet_robots.panda.panda_sim_grasp import rp, pandaEndEffectorIndex, pandaNumDofs, jointPositions

from ompl_demo import StateChecker, create_plan

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

def set_ja_goal(ja, robotId):
    assert len(ja) == 7
    p.setJointMotorControlArray(robotId, jointIndices=[0,1,2,3,4,5,6], controlMode=p.POSITION_CONTROL, targetPositions=ja)


def get_ik_sol(pos, orient, ll, ul, jr, robotId):
    return p.calculateInverseKinematics(robotId, pandaEndEffectorIndex, pos, orient, ll, ul, jr, rp, maxNumIterations=1000)


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


def execute_plan(pl, robotId):
    for i in range(len(pl)):
        set_ja_goal(pl[i], robotId)
        p.stepSimulation()
        time.sleep(1./240.)

def wait(n):
    for i in range(n):
        p.stepSimulation()
        time.sleep(1./240.)

def create_env(with_block, bad_start=False):
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())


    p.setGravity(0, 0, -9.81)

    planeId = p.loadURDF("plane.urdf")
    tableId = p.loadURDF("table/table.urdf")

    aabbmin, aabbmax = p.getAABB(tableId)
    start_pos = [aabbmin[0]+0.2,0,aabbmax[2]]
    start_ori = p.getQuaternionFromEuler([0,0,0])

    if with_block:
        blockId = p.loadURDF("block.urdf", [0,0,aabbmax[2]], start_ori)


    robotId = p.loadURDF("franka_panda/panda.urdf", start_pos, start_ori, useFixedBase=True)

    # RESET THE ENV.

    if bad_start:
        initial_position = [0, 0, 0, 0, 0, 0, 0, 0.02, 0.02]
    else:
        initial_position = INITIAL_POSITION

    index=0
    for j in range(p.getNumJoints(robotId)):
        p.changeDynamics(robotId, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(robotId, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType == p.JOINT_PRISMATIC):
            p.resetJointState(robotId, j, initial_position[index])
            # print(index)
            index=index+1
        if (jointType == p.JOINT_REVOLUTE):
            p.resetJointState(robotId, j, initial_position[index]) 
            index=index+1

    bounds = get_joint_bounds(robotId, physicsClient)
    ll = [l[0] for l in bounds]
    ul = [l[1] for l in bounds]
    jr = [l[1] - l[0] for l in bounds]

    tableheight = aabbmax[2]

    return physicsClient, robotId, ll, ul, jr, bounds, tableheight


def open_gripper(robotId):
    print("opening")
    for i in range(100):
        p.setJointMotorControlArray(robotId, jointIndices=[9,10], controlMode=p.POSITION_CONTROL, targetPositions=[0.04, 0.04])
        p.stepSimulation()
        time.sleep(1./240.)

def close_gripper(robotId):
    print("closing")
    for i in range(100):
        p.setJointMotorControlArray(robotId, jointIndices=[9,10], controlMode=p.POSITION_CONTROL, targetPositions=[0.01, 0.01])
        p.stepSimulation()
        time.sleep(1./240.)


def main():
    simId, robotId, ll, ul, jr, bounds, tableheight = create_env(True)

    # Make sure our initial position is valid...
    checker = StateChecker()
    if not checker.is_state_valid(INITIAL_POSITION[:7]):
        raise ValueError("NOT A VALID STARTING POSITION")


    # We want the gripper facing downwards.
    down = [1, 0, 0, 0]


    # Move the arm directly above the block.
    p1 = [0, 0, tableheight+0.2]
    p1_ja = get_ik_sol(p1, down, ll, ul, jr, robotId)
    p1_plan = create_plan(INITIAL_POSITION[:7], p1_ja[:7], joint_limits=bounds, checker=checker)
    execute_plan(p1_plan, robotId)

    wait(500)

    open_gripper(robotId)

    wait(300)

    # Move the arm down to the block.
    p2 = [0, 0, tableheight+0.02]
    p2_ja = get_ik_sol(p2, down, ll, ul, jr, robotId)
    p1p2_plan = create_plan(p1_ja[:7], p2_ja[:7], joint_limits=bounds, checker=checker)
    execute_plan(p1p2_plan, robotId)

    wait(500)

    close_gripper(robotId)

    wait(300)


    p3 = [0, 0, tableheight+0.2]
    p3_ja = get_ik_sol(p3, down, ll, ul, jr, robotId)
    p2p3_plan = create_plan(p2_ja[:7], p3_ja[:7], joint_limits=bounds, checker=checker)
    execute_plan(p2p3_plan, robotId)

    wait(500)

    p4 = [0, 0.2, tableheight+0.2]
    p4_ja = get_ik_sol(p4, down, ll, ul, jr, robotId)
    p3p4_plan = create_plan(p3_ja[:7], p4_ja[:7], joint_limits=bounds, checker=checker)
    execute_plan(p3p4_plan, robotId)

    wait(500)

    p5 = [0, 0.2, tableheight+0.02]
    p5_ja = get_ik_sol(p5, down, ll, ul, jr, robotId)
    p4p5_plan = create_plan(p4_ja[:7], p5_ja[:7], joint_limits=bounds, checker=checker)
    execute_plan(p4p5_plan, robotId)

    wait(500)

    open_gripper(robotId)

    wait(300)

    p6 = [0, 0.2, tableheight+0.2]
    p6_ja = get_ik_sol(p6, down, ll, ul, jr, robotId)
    p5p6_plan = create_plan(p5_ja[:7], p6_ja[:7], joint_limits=bounds, checker=checker)
    execute_plan(p5p6_plan, robotId)



if __name__ == "__main__":
    main()

