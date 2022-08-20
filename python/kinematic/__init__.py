from sympy import *

# d v a alpha o
def dkin(Robot: Matrix) -> Matrix:
    """The Direct Kinematics function in which
    outputs the matrix T by a given matrix with
    the columns being: d, v, a, alpha and offset.
    Any column added will be ignored.
    
    Returns:
        T: A simplified version of the direct 
        kinematic matrix computed by the Robot given.
    """
    T = eye(4)
    for row in range(Robot.rows):
        T = T * Matrix([[ cos(Robot[row, 1]+Robot[row, 4]), -sin(Robot[row, 1]+Robot[row, 4])*cos(Robot[row, 3]),  sin(Robot[row, 1]+Robot[row, 4])*sin(Robot[row, 3]), Robot[row, 2]*cos(Robot[row, 1])],
                        [ sin(Robot[row, 1]+Robot[row, 4]),  cos(Robot[row, 1]+Robot[row, 4])*cos(Robot[row, 3]), -cos(Robot[row, 1]+Robot[row, 4])*sin(Robot[row, 3]), Robot[row, 2]*sin(Robot[row, 1])],
                        [                                0,                                   sin(Robot[row, 3]),                                   cos(Robot[row, 3]),                    Robot[row, 0]],
                        [                                0,                                                    0,                                                    0,                                1]])
    return nsimplify(T, tolerance=1e-10, rational=False)

def geometric_jacobian(): pass