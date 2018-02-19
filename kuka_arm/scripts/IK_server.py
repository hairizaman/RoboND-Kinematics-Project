#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from numpy import*
from numpy.linalg import inv

## MATRIX UTILITIES

# Build transformation matrix from DH parameters
def DHMatrix(alpha,a,d,theta):
    return matrix([[cos(theta), -sin(theta), 0, a],
                   [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                   [0, 0, 0, 1]])


# Build transformation matrix given configuration vector "q" and start and end coordinate frame indices
def KR210ForwardMatrix(q,alpha,a,d,startIdx,endIdx):

    theta = [q[0], q[1]-pi/2, q[2], q[3], q[4], q[5], 0]; # Includes correction to q2 (or q[1])

    # Loop through all the selected indices
    M = eye(4);
    for idx in range(startIdx,endIdx):
        M = M*DHMatrix(alpha[idx],a[idx],d[idx],theta[idx])
    return M;

# Correction matrix converting from Gazebo coordinates to the DH parameter coordinates
# XG = ZGazebo , YG = -YGazebo , ZG = XGazebo
def RotCorr():
    return matrix([[0, 0, 1],[0, -1, 0],[1, 0, 0]])

# Elementary rotation matrices
def RotX(theta):
    return matrix([[1,0,0],[0,cos(theta),-sin(theta)],[0,sin(theta),cos(theta)]])
def RotY(theta):
    return matrix([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
def RotZ(theta):
    return matrix([[cos(theta),sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])

## END UTILITIES


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here

        # Define all constant DH parameters for the KR210
        # In this case, it's all parameters except theta, since all joints are revolute
        alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0];
        a = [0, 0.35, 1.25, -0.054, 0, 0, 0];
        d = [0.75, 0, 0, 1.5, 0, 0, 0.303];

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here

            # Find the orientation of the end effector in DH coordinates
            Rrpy = RotZ(yaw)*RotY(pitch)*RotX(roll)*RotCorr();
            nx = Rrpy[0,2];
            ny = Rrpy[1,2];
            nz = Rrpy[2,2];

            # Find the wrist center position
            wristLength = d[5]+d[6];
            wx = px - wristLength*nx;
            wy = py - wristLength*ny;
            wz = pz - wristLength*nz;

            # Geometric IK for joints 1-3
            d1 = d[0];
            a1 = a[1];
            a2 = a[2];
            a3 = a[3]; 
            d4 = d[3];
            d4_eff = sqrt(d4*d4 + a3*a3);

            # Theta1 is defined exclusively by the angle on the X-Y Plane
            theta1 = arctan2(wy,wx);

            # Theta2 and Theta3 are interdependent and require some trigonometry.
            # Refer to the README file for a schematic.
            xRef = sqrt(wx*wx + wy*wy) - a1;
            yRef = wz - d1;
            dRef = sqrt(xRef*xRef + yRef*yRef);
            beta = arctan2(yRef,xRef);

            angleA = arccos((dRef*dRef + a2*a2 - d4_eff*d4_eff)/(2*a2*dRef))
            theta2 = (pi/2) - angleA - beta;

            angleB = arccos((a2*a2 + d4_eff*d4_eff - dRef*dRef)/(2*a2*d4_eff))
            theta3 = (pi/2) - angleB - arctan2(-a3,d4);

            # Inverse Orientation for Joints 4-6
            q = [theta1, theta2, theta3, 0, 0, 0] # Last 3 values don't matter yet
            R0_3 = KR210ForwardMatrix(q,alpha,a,d,0,4)[0:3,0:3] # Do not need 4th dimension
            R3_6 = R0_3.T * Rrpy;
            theta4, theta5, theta6 = tf.transformations.euler_from_matrix(R3_6,axes='rzyz')
            theta5 = -theta5; # Since the rotation is actually z, -y, z

            # Handle multiple theta solutions
            # If theta4 is near inverted (pi), flip it by an offset of pi to be near zero
            # Note this will require theta5 to be reversed in sign
            if (theta4 > 0) and ((pi-theta4) < theta4):
                theta4 = theta4 - pi;
                theta5 = -theta5;
            elif (theta4 < 0) and ((theta4+pi) < -theta4):
                theta4 = theta4 + pi;
                theta5 = -theta5;

            # Do the same for theta6, which doesn't affect other angles
            if (theta6 > 0) and ((pi-theta6) < theta6):
                theta6 = theta6 - pi;
            elif (theta6 < 0) and ((theta6+pi) < -theta6):
                theta6 = theta6 + pi;
 
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
