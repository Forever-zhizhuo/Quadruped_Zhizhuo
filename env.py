import pybullet as p
import pybullet_data as pd
import numpy as np
import time
import baixian_straight
import baixian_turn
import inverse_kinematic

t = 0

class Robo_env():
    def __init__(self):
        self.RoboID = None
        self.sleep_t = 1. / 240  # decrease the value if it is too slow.
        self.maxVelocity = 10
        self.force = 100
        self.n_sim_steps = 1
        self.startPos = [0, 0, 0.5]
        self.startOri = [np.pi / 2, 0, 0]
        self.initial_action = []
        self.jointIds = []
        self.reset()

    def step_forward(self):

        R_x = -0.05
        L_x = 0.05
        FR_t = t
        RL_t = t + 0.75
        FL_t = t + 0.50
        RR_t = t + 0.25
        print("the time is {}".format(t))

        FR_z, FR_y = baixian_straight.positive_calculation(self, FR_t)
        list_1 = inverse_kinematic.inverse_kinematic_zzz(R_x, FR_y, FR_z)

        RL_z, RL_y = baixian_straight.positive_calculation(self, RL_t)
        list_2 = inverse_kinematic.inverse_kinematic_zzz(L_x, RL_y, RL_z)

        FL_z, FL_y = baixian_straight.positive_calculation(self, FL_t)
        list_3 = inverse_kinematic.inverse_kinematic_zzz(L_x, FL_y, FL_z)

        RR_z, RR_y = baixian_straight.positive_calculation(self, RR_t)
        list_4 = inverse_kinematic.inverse_kinematic_zzz(R_x, RR_y, RR_z)

        action = list_1 + list_3 + list_4 + list_2

        for j in range(12):
            targetPos = float(action[j])
            targetPos = self.jointDirections[j] * targetPos + self.jointOffsets[j]
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[j],
                                    targetPosition=targetPos,
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
        for _ in range(self.n_sim_steps):
            p.stepSimulation()
            time.sleep(1. / 50)

    def step_left(self):

        FR_z = 0
        FL_z = 0
        RL_z = 0
        RR_z = 0
        FR_t = t
        RL_t = t
        FL_t = t + 0.50
        RR_t = t + 0.50
        print("the time is {}".format(t))

        FR_x, FR_y = baixian_turn.positive_calculation(self, FR_t)
        list_1 = inverse_kinematic.inverse_kinematic_zzz(FR_x, FR_y, FR_z)

        RL_x, RL_y = baixian_turn.negative_calculation(self, RL_t)
        list_2 = inverse_kinematic.inverse_kinematic_zzz(RL_x, RL_y, RL_z)

        FL_x, FL_y = baixian_turn.positive_calculation(self, FL_t)
        list_3 = inverse_kinematic.inverse_kinematic_zzz(FL_x, FL_y, FL_z)

        RR_x, RR_y = baixian_turn.negative_calculation(self, RR_t)
        list_4 = inverse_kinematic.inverse_kinematic_zzz(RR_x, RR_y, RR_z)

        action = list_1 + list_3 + list_4 + list_2

        for j in range(12):
            targetPos = float(action[j])
            targetPos = self.jointDirections[j] * targetPos + self.jointOffsets[j]
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[j],
                                    targetPosition=targetPos,
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
        for _ in range(self.n_sim_steps):
            p.stepSimulation()
            time.sleep(1. / 50)

    def step_right(self):

        FR_z = 0
        FL_z = 0
        RL_z = 0
        RR_z = 0

        FR_t = t
        RL_t = t
        FL_t = t + 0.50
        RR_t = t + 0.50
        print("the time is {}".format(t))

        FR_x, FR_y = baixian_turn.negative_calculation(self, FR_t)
        list_1 = inverse_kinematic.inverse_kinematic_zzz(FR_x, FR_y, FR_z)

        RL_x, RL_y = baixian_turn.positive_calculation(self, RL_t)
        list_2 = inverse_kinematic.inverse_kinematic_zzz(RL_x, RL_y, RL_z)

        FL_x, FL_y = baixian_turn.negative_calculation(self, FL_t)
        list_3 = inverse_kinematic.inverse_kinematic_zzz(FL_x, FL_y, FL_z)

        RR_x, RR_y = baixian_turn.positive_calculation(self, RR_t)
        list_4 = inverse_kinematic.inverse_kinematic_zzz(RR_x, RR_y, RR_z)

        action = list_1 + list_3 + list_4 + list_2

        for j in range(12):
            targetPos = float(action[j])
            targetPos = self.jointDirections[j] * targetPos + self.jointOffsets[j]
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[j],
                                    targetPosition=targetPos,
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
        for _ in range(self.n_sim_steps):
            p.stepSimulation()
            time.sleep(1. / 50)

    def step_backward(self):

        R_x = -0.05
        L_x = 0.05
        FR_t = t
        RL_t = t + 0.75
        FL_t = t + 0.50
        RR_t = t + 0.25
        print("the time is {}".format(t))

        FR_z, FR_y = baixian_straight.negative_calculation(self, FR_t)
        list_1 = inverse_kinematic.inverse_kinematic_zzz(R_x, FR_y, FR_z)

        RL_z, RL_y = baixian_straight.negative_calculation(self, RL_t)
        list_2 = inverse_kinematic.inverse_kinematic_zzz(L_x, RL_y, RL_z)

        FL_z, FL_y = baixian_straight.negative_calculation(self, FL_t)
        list_3 = inverse_kinematic.inverse_kinematic_zzz(L_x, FL_y, FL_z)

        RR_z, RR_y = baixian_straight.negative_calculation(self, RR_t)
        list_4 = inverse_kinematic.inverse_kinematic_zzz(R_x, RR_y, RR_z)

        action = list_1 + list_3 + list_4 + list_2

        for j in range(12):
            targetPos = float(action[j])
            targetPos = self.jointDirections[j] * targetPos + self.jointOffsets[j]
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[j],
                                    targetPosition=targetPos,
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
        for _ in range(self.n_sim_steps):
            p.stepSimulation()
            time.sleep(1. / 50)

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        planeId = p.loadURDF("plane.urdf")  # URDF Id = 0
        self.RoboID = p.loadURDF("laikago/laikago.urdf", self.startPos,
                                 p.getQuaternionFromEuler(self.startOri))  # URDF Id = 1

        for zzz in range(p.getNumJoints(self.RoboID)):
            joint_id = zzz
            joint_info = p.getJointInfo(self.RoboID, joint_id)
            print(joint_info)

        self.jointOffsets = []
        self.jointDirections = [-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1]

        for i in range(4):
            self.jointOffsets.append(0)
            self.jointOffsets.append(-0.7)
            self.jointOffsets.append(0.7)

        for j in range(p.getNumJoints(self.RoboID)):
            p.changeDynamics(self.RoboID, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.RoboID, j)
            jointName = info[1]
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                self.jointIds.append(j)

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pd.getDataPath())  # optionally
    env1 = Robo_env()

    while t<=5:
        env1.step_forward()
        t = t + 0.01
    
    time.sleep(1)

    while t<=10:
        env1.step_left()
        t = t + 0.01

    time.sleep(1)

    while t<=15:

        env1.step_right()
        t = t + 0.01

    time.sleep(1)

    while 1:
        env1.step_backward()
        t = t + 0.01