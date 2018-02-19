from sympy import *
from numpy import *
from numpy.linalg import inv
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

## UTILITIES TO COPY OVER 

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



def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!

    # Define all constant DH parameters for the KR210
    # In this case, it's all parameters except theta, since all joints are revolute
    alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0];
    a = [0, 0.35, 1.25, -0.054, 0, 0, 0];
    d = [0.75, 0, 0, 1.5, 0, 0, 0.303];
      
    # Find the Euler Angles of the requested orientation
    q = [orientation.x,orientation.y,orientation.z,orientation.w]
    eul = tf.transformations.euler_from_quaternion(q)
    roll = eul[0];
    pitch = eul[1];
    yaw = eul[2];

    # Find the orientation of the end effector in DH coordinates
    Rrpy = RotZ(yaw)*RotY(pitch)*RotX(roll)*RotCorr();
    nx = Rrpy[0,2];
    ny = Rrpy[1,2];
    nz = Rrpy[2,2];

    # Find the wrist center position
    wristLength = d[5]+d[6];
    wx = position.x - wristLength*nx;
    wy = position.y - wristLength*ny;
    wz = position.z - wristLength*nz;

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

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    # Get the Forward Kinematics matrices
    q = [theta1,theta2,theta3,theta4,theta5,theta6];
    M_ee = KR210ForwardMatrix(q,alpha,a,d,0,7); # End effector: From base to gripper

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [M_ee[0,3],M_ee[1,3],M_ee[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1
    test_code(test_cases[test_case_number])

