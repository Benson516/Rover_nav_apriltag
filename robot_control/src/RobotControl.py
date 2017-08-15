#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
from math import atan2
from ShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController
from wayPointFollowing import wayPointFollowing

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        self.pos_goal = pos_goal

        # YOUR CODE AFTER THIS
        #-------------------------------------------#
        self.time = rospy.get_time()
        self.controlOut = (0.0, 0.0, False)
        self.count_noMeasurement = 0

        #-------------------------------------------#
        self.markers = world_map
        self.idx_target_marker = 0

        # Calculate the optimal path
        # From pos_init to pos_goal
        self.path_2D = dijkstras(occupancy_map,x_spacing,y_spacing, pos_init, pos_goal)
        self.idx_path = 0
        self.size_path = self.path_2D.shape[0]
        print "path.shape[0]",self.size_path
        # Generate the 3D path (include "theta")
        self.path = np.zeros((self.size_path,3))
        theta = 0.0
        for idx in range(self.size_path-1):
            delta = self.path_2D[(idx+1),:] - self.path_2D[idx,:]
            theta = atan2(delta[1],delta[0])
            if theta > np.pi:
                theta -= np.pi*2
            elif theta < -np.pi:
                theta += np.pi*2
            self.path[idx,:] = np.concatenate((self.path_2D[idx,:], np.array([theta])),axis=1)
        self.path[self.size_path-1,0:2] = self.path_2D[self.size_path-1,0:2]
        self.path[self.size_path-1,2] = pos_goal[2] # theta
        #
        self.path[0,2] = pos_init[2] # theta
        print "3D path:"
        print self.path

        # Uncomment as completed
        # Kalman filter
        self.kalman_filter = KalmanFilter(world_map)
        self.kalman_filter.mu_est = pos_init # 3*pos_init # For test

        # Differential drive
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        self.wayPointFollowing = wayPointFollowing(max_speed, max_omega)

        #
        self.task_done = False

    def process_measurements(self):
        """
        YOUR CODE HERE
        This function is called at 60Hz
        """
        meas = self.ros_interface.get_measurements()
        imu_meas = self.ros_interface.get_imu()

        # print 'meas',meas
        print 'imu_meas',imu_meas

        """
        # Control the robot to track the tag
        if (meas is None) or (meas == []):
            # stop
            # self.ros_interface.command_velocity(0.0,0.0)
            #
            if self.count_noMeasurement > 30:
                # Actually the ros_interface will stop the motor itself if we don't keep sending new commands
                self.ros_interface.command_velocity(0.0,0.0)
            else:
                # Keep the old command
                self.ros_interface.command_velocity(self.controlOut[0],self.controlOut[1])
                self.count_noMeasurement += 1
        else:
            print 'meas',meas
            self.count_noMeasurement = 0
            # Thew differential drive controller that lead the robot to the tag
            self.controlOut = self.diff_drive_controller.compute_vel((-1)*np.array(meas[0][0:3]),np.array([0.0,0.0,0.0]))
            self.ros_interface.command_velocity(self.controlOut[0],self.controlOut[1])

        (v,omega,done) = (self.controlOut[0], self.controlOut[1],False )
        """

        """
        if (rospy.get_time() - self.time) < 1.0:
            self.ros_interface.command_velocity(0.0, 3.0)
            # Linear velocity = 0.3 m/s, Angular velocity = 0.5 rad/s
        else:
            self.ros_interface.command_velocity(0.0,0.0)
        """
        """
        (v,omega,done) = (0.0, 0.0, False)
        self.ros_interface.command_velocity(v, omega)
        """

        """
        # Directly move to the goal position
        if self.controlOut[2]: # done
            self.ros_interface.command_velocity(0.0, 0.0)
        else:
            self.controlOut = self.wayPointFollowing.compute_vel(self.kalman_filter.mu_est, self.pos_goal )
            # self.controlOut = self.diff_drive_controller.compute_vel(self.kalman_filter.mu_est, self.pos_goal )
            if self.controlOut[2]: # done
                self.ros_interface.command_velocity(0.0, 0.0)
            else:
                self.ros_interface.command_velocity(self.controlOut[0], self.controlOut[1])
        """

        """
        #----------------------------------------#
        # Switch the targets (way-point of the optimal path) and do the position control
        # if self.controlOut[2] and self.idx_path == self.size_path-1: # all done
        if self.idx_path == self.size_path-1 and self.controlOut[0] < 0.05 and self.controlOut[1] < 0.1 : # all done
            self.ros_interface.command_velocity(0.0, 0.0)
        else:
            self.controlOut = self.diff_drive_controller.compute_vel(self.kalman_filter.mu_est, self.path[self.idx_path,:] )
            self.ros_interface.command_velocity(self.controlOut[0], self.controlOut[1])
            if self.controlOut[2] and self.idx_path < self.size_path-1: # way-point done
                self.idx_path += 1
        #----------------------------------------#
        """

        #----------------------------------------#
        print "self.idx_path",self.idx_path
        # Switch the targets (way-point of the optimal path) and do the position control
        # way-point following (no reducing speeed when reach a way-point)
        # if self.controlOut[2] and self.idx_path == self.size_path-1: # all done
        if self.task_done: # all done
            self.ros_interface.command_velocity(0.0, 0.0)
        elif self.idx_path == self.size_path-1: # The last one
            # Change the threshold
            self.diff_drive_controller.threshold = 0.02 # 3 cm
            # Using diff_drive_controller to reach the goal position with right direction
            self.controlOut = self.diff_drive_controller.compute_vel(self.kalman_filter.mu_est, self.path[self.idx_path,:] )
            # if self.controlOut[0] < 0.05 and self.controlOut[1] < 0.1 : # all done
            if self.controlOut[2]: # all done
                self.ros_interface.command_velocity(0.0, 0.0)
                self.task_done = True # all done
            else:
                self.ros_interface.command_velocity(self.controlOut[0], self.controlOut[1])
        else:
            # The way-points, using wayPointFollowing to trace the trajectory without pausing

            if self.idx_path == self.size_path-2: # The last 2nd one
                self.controlOut = self.diff_drive_controller.compute_vel(self.kalman_filter.mu_est, self.path[self.idx_path,:] )
            else:
                self.controlOut = self.wayPointFollowing.compute_vel(self.kalman_filter.mu_est, self.path[self.idx_path,:] )

            # self.controlOut = self.wayPointFollowing.compute_vel(self.kalman_filter.mu_est, self.path[self.idx_path,:] )
            self.ros_interface.command_velocity(self.controlOut[0], self.controlOut[1])
            if self.controlOut[2] and self.idx_path < self.size_path-1: # way-point done
                self.idx_path += 1
        #----------------------------------------#
        # self.time = rospy.get_time()
        # print "self.time",self.time

        # Kalman filter
        self.kalman_filter.step_filter(self.controlOut[0], (-1)*imu_meas, meas, rospy.get_time())
        print "mu_est",self.kalman_filter.mu_est


        return

def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
