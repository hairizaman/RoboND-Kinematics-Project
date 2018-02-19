# Project: Kinematics Pick & Place
### Submission for Udacity Robotics Software Engineer Nanodegree Program
### Sebastian Castro - 2018


[//]: # (Image References)

[coordinate_correction]: ./misc_images/coordinate_correction.png
[kr210_forward_kinematics]: ./misc_images/kr210_forward_kinematics.jpg
[kr210_geometric_ik_1]: ./misc_images/kr210_geometric_ik_1.png
[kr210_geometric_ik_2]: ./misc_images/kr210_geometric_ik_2.png
[motionplanning]: ./misc_images/motionplanning1.PNG

---

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
_q2_ since its zero configuration corresponds to an angle of _pi/2_ in the
Gazebo simulation.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->G | 0 | 0 | 0.303 | 0

Finally, there needed to be an additional correction matrix since the 
end effector axis orientation in our schematic is different from that 
in Gazebo. The URDF file aligned the global world coordinate frame with
each joint in zero configuration, whereas we had other rotations that
made the DH parameters easier to find.

![Coordinate system correction][coordinate_correction]


## Inverse Kinematics

The Inverse Kinematics (IK) problem was solved analytically since the 
KR210 manipulator has 6 degrees of freedom and a spherical wrist.

### Step 1 - Find the wrist center position
Given the requested orientation (roll-pitch-yaw) of the end effector, 
as well as the length of the wrist, we can back out the location of the
wrist center. This wrist center location can be used to calculate the 
first 3 angles of the manipulator.

```
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
```

### Step 2 - Find the first 3 angles
Using the robot geometry and the location of the wrist center, we can 
determine the angles _theta1_ through _theta3_. 

The angle _theta1_ is the only joint angle that affects the base 
rotation about the world Z axis, so it can be found by taking the X and Y
components of the wrist center.

![KR210 Geometric IK for theta1][kr210_geometric_ik_1]

The angles _theta2_ and _theta3_ can then be found by taking the manipulator 
geometry in the robot plane and applying the law of cosines. The schematic 
below shows the necessary equations.

![KR210 Geometric IK for theta2 and theta3][kr210_geometric_ik_2]

# Step 3 - Find the last 3 angles
Now that we have the angles _theta1_ through _theta3_, we can use forward 
kinematics to get the homogeneous matrix of the robot at the wrist center. 
Then, we can invert this matrix to find the transformation matrix from the 
wrist center to the end effector.

Then, we can take this rotation matrix and find the 3 spherical wrist joint 
angles needed to complete the inverse kinematics. This was done using the 
`euler_from_matrix` function, with knowledge that the spherical wrist 
(as defined in our convention) is a rotating Z-Y-Z wrist. Also, we also 
need to account for the fact that the Y axis in our convention is in the 
opposite direction.

```
# Inverse Orientation for Joints 4-6
q = [theta1, theta2, theta3, 0, 0, 0] # Last 3 values don't matter yet
R0_3 = KR210ForwardMatrix(q,alpha,a,d,0,4)[0:3,0:3] # Do not need 4th dimension
R3_6 = R0_3.T * Rrpy;
theta4, theta5, theta6 = tf.transformations.euler_from_matrix(R3_6,axes='rzyz')
theta5 = -theta5; # Since the rotation is actually z, -y, z
```

At first, the wrist was rotating wildly because the acceptable values ranged 
from +pi to -pi. We resolved this issue by ensuring that the _theta4_ and 
_theta6_ angles can only take values between +pi/2 and -pi/2. This way, 
the wrist joints are prevented from continuously rotating along the planned 
motion trajectory.

As shown in the code snippet below, flipping the _theta4_ wrist angle requires 
theta5 to be flipped in the opposite direction to maintain the same wrist 
orientation as calculated directly from the matrix.

```
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
```

## Results and Implementation Notes
In contrast to the recommended workflow, we decided to use _numpy_ instead of 
_sympy_ to remove the need for symbolic math. Therefore, you will see that 
the forward kinematics are defined in a separate utility function called 
`KR210ForwardMatrix`, which is inverted on the fly during IK computation using 
the numpy `inv` function.

The inverse kinematics solution adequately solved all test cases within a 
few centimeters of error. Unfortunately, my Virtual Machine environment 
prevented me from looping through all test cases continuously, as 
respawning a new object after the first task was complete generated an 
error message about the `trajectory_sampler` process dying.

![KR210 Manipulator in action][motionplanning]
