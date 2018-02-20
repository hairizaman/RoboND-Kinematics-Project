from numpy import *
from sympy import *
from tf import transformations

# Build transformation matrix from DH parameters
def DHMatrix(alpha,a,d,theta):
    return matrix([[cos(theta), -sin(theta), 0, a],
                   [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                   [0, 0, 0, 1]])


# Build transformation matrix given configuration vector "q" and start and end coordinate frame indices
def KR210ForwardMatrix(q,params,startIdx,endIdx):

    theta = [q[0], q[1]-pi/2, q[2], q[3], q[4], q[5], 0]; # Includes correction to q2 (or q[1])

    # Loop through all the selected indices
    M = eye(4);
    for idx in range(startIdx,endIdx):
        M = M*DHMatrix(params["alpha"][idx],params["a"][idx],params["d"][idx],theta[idx])
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


    # Define symbols for all DH parameters
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7');
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7');
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8');
    q1, q2, q3, q4, q5, q6 = symbols('q1:7');

    # Package the DH parameters
    dhParams = { 
               "alpha":[alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6],
               "a": [a0,a1,a2,a3,a4,a5,a6],
               "d": [d1,d2,d3,d4,d5,d6,d7],
               }
    q = [q1,q2,q3,q4,q5,q6,0];

    # Define the a, alpha, and d values to substitute based on the URDF file and derived DH parameters
    subsDict = {a0: 0, a1: 0.35, a2: 1.25, a3: -0.054, a4: 0, a5: 0, a6: 0, 
                alpha0: 0, alpha1: -pi/2, alpha2: 0, alpha3: -pi/2, alpha4: pi/2, alpha5: -pi/2, alpha6: 0, 
                d1: 0.75, d2: 0, d3: 0, d4: 1.5, d5: 0, d6: 0, d7: 0.303};

    # Get each individual matrix
    M1_0 = KR210ForwardMatrix(q,dhParams,0,1)
    print("M1_0")
    print(simplify(M1_0))
    print(simplify(M1_0.evalf(subs=subsDict)))

    M2_1 = KR210ForwardMatrix(q,dhParams,1,2)
    print("M2_1")
    print(simplify(M2_1))
    print(simplify(M2_1.evalf(subs=subsDict)))

    M3_2 = KR210ForwardMatrix(q,dhParams,2,3)
    print("M3_2")
    print(simplify(M3_2))
    print(simplify(M3_2.evalf(subs=subsDict)))

    M4_3 = KR210ForwardMatrix(q,dhParams,3,4)
    print("M4_3")
    print(simplify(M4_3))
    print(simplify(M4_3.evalf(subs=subsDict)))

    M5_4 = KR210ForwardMatrix(q,dhParams,4,5)
    print("M5_4")
    print(simplify(M5_4))
    print(simplify(M5_4.evalf(subs=subsDict)))

    M6_5 = KR210ForwardMatrix(q,dhParams,5,6)
    print("M6_5")
    print(simplify(M6_5))
    print(simplify(M6_5.evalf(subs=subsDict)))

    MG_6 = KR210ForwardMatrix(q,dhParams,6,7)
    print("MG_6")
    print(simplify(MG_6))
    print(simplify(MG_6.evalf(subs=subsDict)))

    # Get the full Forward Kinematics matrix
    # Define a configuration vector
    q = [0,0,0,0,0,0];
    #q = [2.71,-0.49,-0.49,5.31,-0.78,-0.96];
    dhParams = { 
               "alpha":[0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0],
               "a": [0, 0.35, 1.25, -0.054, 0, 0, 0],
               "d": [0.75, 0, 0, 1.5, 0, 0, 0.303],
               }
    M = KR210ForwardMatrix(q,dhParams,0,7);
    print("Homogeneous matrix from base to end effector")
    print(M)
 





