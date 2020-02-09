import sys
import numpy as np
import math
class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):
        """
        TODO : Initialize Motion Model parameters here
        """
    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here
        """
        alpha=[0.1,0.01,0.3,0.3]
        # alpha=[0.1,0.05,1,1]
        x_t1=np.zeros(x_t0.shape)
        deltaRot1 = math.atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t1[2]
        deltaTrans = math.sqrt((u_t1[0] - u_t0[0]) ** 2 + (u_t1[1] - u_t0[1]) ** 2)
        deltaRot2 = u_t1[2] - u_t0[2] - deltaRot1

        trueRot1 = deltaRot1 - np.random.normal(0, alpha[0] * abs(deltaRot1) + alpha[1] * abs(deltaTrans))
        trueTrans= deltaTrans- np.random.normal(0, alpha[2] * abs(deltaTrans)+ alpha[3] * (
       abs(deltaRot1) + abs(deltaRot2)))
        trueRot2 = deltaRot2 - np.random.normal(0, alpha[0] * abs(deltaRot2) + alpha[1] * abs(deltaTrans))

        x_t1[0] = x_t0[0] + trueTrans * math.cos(x_t0[2] + trueRot1)
        x_t1[1] = x_t0[1] + trueTrans * math.sin(x_t0[2] + trueRot1)
        x_t1[2] = x_t0[2] + trueRot1 + trueRot2
        return x_t1
