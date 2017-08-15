#!/usr/bin/python

import numpy as np
from math import sqrt,atan2

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        #

        self.pole_1 = 0.5 # 0.3 # 1.0 # 0.3
        self.pole_2 = 0.7 # 0.7 # 1.2 # 0.5
        self.pole_3 = 0.7 # 0.7 # 1.5 # 0.5
        self.kp = self.pole_1
        self.ka = (self.pole_2 + self.pole_3) + self.kp
        self.kb = self.pole_2*self.pole_3/self.kp

        """
        self.kp = 0.5
        self.ka = 0.01 + self.kp
        self.kb = 12.0
        """
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        #
        self.threshold = 0.15 # 0.03 # 0.1

    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        """
        err = goal - state
        #
        rho = sqrt(err[0]**2 + err[1]**2)
        alpha = -err[2] + atan2(err[1],err[0])
        beta = -err[2] - alpha
        # beta = -atan2(err[1],err[0])
        # alpha = -beta - err[2]
        """
        #
        state = state.reshape(3,1)
        # print state.shape
        goal = goal.reshape(3,1)
        # print goal.shape

        err = state - goal
        rho = sqrt(err[0,0]**2 + err[1,0]**2)
        # beta = atan2(-err[1,0],-err[0,0])
        beta = atan2(-err[1,0],-err[0,0]) - goal[2,0]
        pi_2 = 2.0*np.pi
        if beta > np.pi:
            beta -= pi_2
        elif beta < -np.pi:
            beta += pi_2
        alpha = beta - err[2,0]
        # print 'err =',err
        # print 'rho =',rho,'alpha =',alpha,'beta =',beta

        # Control law
        v = self.kp*rho;
        omega = self.ka*alpha + self.kb*beta

        # Saturation
        if v > self.MAX_SPEED:
            v = self.MAX_SPEED
        if omega > self.MAX_OMEGA:
            omega = self.MAX_OMEGA
        elif omega < -self.MAX_OMEGA:
            omega = -self.MAX_OMEGA

        # Done?
        if rho <= self.threshold:
            done = True
        else:
            done = False
        #
        # (v,omega,done) = (0.0,0.0,False)
        return (v,omega, done)
