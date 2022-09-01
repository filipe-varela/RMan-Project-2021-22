import sympy as sp
import symengine as se
from sympy import cos, sin

# d v a alpha o
def dkin(Robot: sp.Matrix) -> sp.Matrix:
    """The Direct Kinematics function in which
    outputs the matrix T by a given matrix with
    the columns being: [d, v, a, alpha, offset].
    Any column added will be ignored.
    
    Returns:
        T: A simplified version of the direct 
        kinematic matrix computed by the Robot given.
    """
    T = se.eye(4)
    for row in range(Robot.rows):
        d, v, a, alpha, o = Robot[row, 0], Robot[row, 1], Robot[row, 2], Robot[row, 3], Robot[row, 4]
        T = T * se.Matrix([[ cos(v+o), -sin(v+o)*cos(alpha),  sin(v+o)*sin(alpha), a*cos(v+o)],
                           [ sin(v+o),  cos(v+o)*cos(alpha), -cos(v+o)*sin(alpha), a*sin(v+o)],
                           [        0,           sin(alpha),           cos(alpha),          d],
                           [        0,                    0,                    0,          1]])
    return sp.nsimplify(se.sympify(T), tolerance=1e-10, rational=False)
    # T = se.sympify(T)
    # T = sp.Matrix([el.xreplace(Transform(lambda x: round(x, 3), lambda x: isinstance(x, sp.Float))) for row in T for el in row])
    # return sp.Matrix(T)