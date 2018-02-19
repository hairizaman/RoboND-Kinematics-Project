# Robotic arm - Pick & Place project
# Sebastian Castro

[//]: # Image Reference Section
[coordinate_correction]: ./misc_images/coordinate_correction.png
[kr210_forward_kinematics]: ./misc_images/kr210_forward_kinematics.jpg


## Kinematic Analysis

### Forward Kinematics
The first step in forward kinematics was to identify the joints in the 
KR210 manipulator by launching the `forward_kinematics.launch` file and
moving each joint individually.

Then, a schematic of the robot was drawn, setting the coordinate frame
locations such that the DH parameters are easily determined.

![KR210 Forward Kinematics][kr210_forward_kinematics]

The resulting modified DH parameters were then found by looking at the
lengths in the robot URDF file. Note that there is a correction for the angle 
*q2* since its zero configuration corresponds to an angle of *pi/2* in the
Gazebo simulation.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

Finally, there needed to be an additional correction matrix since the 
end effector axis orientation in our schematic is different from that 
in Gazebo. The URDF file aligned the global world coordinate frame with
each joint in zero configuration, whereas we had other rotations that
made the DH parameters easier to find.

![Coordinate system correction][coordinate_correction]



## Inverse Kinematics
Discuss the geometric IK method used
* Gazebo-ROS connection
* Finding theta1-theta3 from wrist center (+schematic)
* Finding the wrist orientation

## Implementation
Describe my use of numpy instead of sympy
Utility functions for matrices

