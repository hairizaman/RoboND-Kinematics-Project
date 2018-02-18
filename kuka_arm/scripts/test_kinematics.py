from numpy import *
from tf import transformations

# Build transformation matrix from DH parameters
def DHMatrix(alpha,a,d,theta):
    return matrix([[cos(theta), -sin(theta), 0, a],
                   [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                   [0, 0, 0, 1]])


# Build transformation matrix given configuration vector "q" and start and end coordinate frame indices
def KR210ForwardMatrix(q,startIdx,endIdx):

    # Define all DH parameters for the KR210
    alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0];
    a = [0, 0.35, 1.25, -0.054, 0, 0, 0];
    d = [0.75, 0, 0, 1.5, 0, 0, 0.303];
    theta = [q[0], q[1]-pi/2, q[2], q[3], q[4], q[5], 0]; # Includes correction to q2 (or q[1])

    # Loop through all the selected indices
    M = eye(4);
    for idx in range(startIdx,endIdx+1):
        M = M*DHMatrix(alpha[idx],a[idx],d[idx],theta[idx])
    return M;

    
# Correction matrix converting from Gazebo coordinates to the DH parameter coordinates
# XG = ZGazebo , YG = -YGazebo , ZG = XGazebo
def CorrectionMatrix():
    return matrix([[0, 0, 1],[0, -1, 0],[1, 0, 0]])

# Elementary rotation matrices
def RotX(theta):
    return matrix([[1,0,0],[0,cos(theta),-sin(theta)],[0,sin(theta),cos(theta)]])
def RotY(theta):
    return matrix([[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]])
def RotZ(theta):
    return matrix([[cos(theta),sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])




if __name__ == "__main__":

    # Define a configuration vector
    q = [0,0,0,0,0,0];
    #q = [2.71,-0.49,-0.49,5.31,-0.78,-0.96];

    # Get the full Forward Kinematics matrix
    M = KR210ForwardMatrix(q,0,6);
    print(M)

    # Test inverse kinematics
    targetPos = matrix([[2],[0],[2]]);
 





