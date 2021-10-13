from ompl import base as ob
from ompl import geometric as og

import pybullet as p
import pybullet_data

from utils import get_self_link_pairs, get_moving_links, pairwise_collision, pairwise_link_collision
from itertools import product

from pybullet_robots.panda.panda_sim_grasp import ll, ul, jr, rp, pandaEndEffectorIndex, pandaNumDofs, jointPositions

def create_state_checker_env():
    simID = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -9.81, simID)

    planeId = p.loadURDF("plane.urdf", physicsClientId=simID)
    tableId = p.loadURDF("table/table.urdf", physicsClientId=simID)

    aabbmin, aabbmax = p.getAABB(tableId, physicsClientId=simID)
    start_pos = [aabbmin[0]+0.2,0,aabbmax[2]]
    start_ori = p.getQuaternionFromEuler([0,0,0])

    # Make the robot in our state checker.
    robotId = p.loadURDF(
        "franka_panda/panda.urdf",
        start_pos,
        start_ori,
        useFixedBase=True,
        physicsClientId=simID
    )

    return simID, robotId, tableId


class StateChecker:
    def __init__(self, with_block=False):
        self.simID, self.robotId, tableId = create_state_checker_env()
        obstacles = [tableId]

        # DON'T WORRY ABOUT THIS SECTION; THIS IS JUST BOILERPLATE FOR GETTING WHICH PAIRS
        # OF LINKS TO CHECK.
        joint_idx = list(range(7))
        self.check_link_pairs = get_self_link_pairs(self.robotId, joint_idx, simID=self.simID)
        moving_links = frozenset(get_moving_links(self.robotId, joint_idx, simID=self.simID))
        moving_bodies = [(self.robotId, moving_links)]
        self.check_body_pairs = list(product(moving_bodies, obstacles))

    def is_state_valid(self, state):
        state_list = [state[i] for i in range(7)]

        # Put our simulation into the specified state.
        for joint, value in zip(range(7), state_list):
            p.resetJointState(self.robotId, joint, value, targetVelocity=0, physicsClientId=self.simID)

        # Check for collisions with the robot links.
        for link1, link2 in self.check_link_pairs:
            if pairwise_link_collision(self.robotId, link1, self.robotId, link2, simID=self.simID):
                return False

        # Check for collisions with the environment.
        for body1, body2 in self.check_body_pairs:
            if pairwise_collision(body1, body2, self.simID):
                return False
        return True


DEFAULT_PLANNING_TIME = 5.0
INTERPOLATE_NUM = 500

def create_plan(current_ja, desired_ja, joint_limits, checker):
    assert len(current_ja) == len(desired_ja)

    n_joints = len(current_ja)

    # The state space we're trying to plan in is JOINT SPACE.
    # We could probably also perform things in SE(3), but maybe not yet.
    state_space = ob.RealVectorStateSpace(n_joints)

    # Describe the general bounds of the optimization problem.
    bounds = ob.RealVectorBounds(n_joints)
    for i, (low, hi) in enumerate(joint_limits):
        bounds.setHigh(i, hi)
        bounds.setLow(i, low)
    state_space.setBounds(bounds)

    # Create the solver. The default planner is RRT, but we can set it to any
    # planning algorithm (or create our own!) via `solver.setPlanner`
    solver = og.SimpleSetup(state_space)
    solver.setStateValidityChecker(ob.StateValidityCheckerFn(checker.is_state_valid))

    # Initialize the start and the goal.
    start, goal = ob.State(state_space), ob.State(state_space)
    for i, (s_i, e_i) in enumerate(zip(current_ja, desired_ja)):
        start[i] = s_i
        goal[i] = e_i
    solver.setStartAndGoalStates(start, goal)

    # Solve the task
    solved = solver.solve(DEFAULT_PLANNING_TIME)
    if not solved:
        raise ValueError("unable to solve the problem")

    # Simplify, essentially tightening the solution.
    solver.simplifySolution()

    # Get the solution, and perform interpolation.
    path = solver.getSolutionPath()
    path.interpolate(INTERPOLATE_NUM)
    path_states = path.getStates()
    path_list = [[state[i] for i in range(n_joints)] for state in path_states]

    return path_list



def main():
    pass


if __name__ == "__main__":
    current_ja = jointPositions[:7]
    desired_ja = [1, 1, 1, 1, 1, 1, 1]

    checker = StateChecker()
    print(checker.is_state_valid(current_ja))
    print(checker.is_state_valid(desired_ja))
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

    plan = create_plan(current_ja, desired_ja, joint_limits=[(-7, 7)]*7, checker=checker)