import symengine as se
from symengine import cos, sin
import sympy as sp

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
    from decimal import Decimal, ROUND_HALF_UP
    T = se.eye(4)
    for row in range(Robot.rows):
        d, v, a, alpha, o = Robot[row, 0], Robot[row, 1], Robot[row, 2], Robot[row, 3], Robot[row, 4]
        T = T * se.Matrix([[ cos(v+o), -sin(v+o)*cos(alpha),  sin(v+o)*sin(alpha), a*cos(v+o)],
                           [ sin(v+o),  cos(v+o)*cos(alpha), -cos(v+o)*sin(alpha), a*sin(v+o)],
                           [        0,           sin(alpha),           cos(alpha),          d],
                           [        0,                    0,                    0,          1]])
    return sp.nsimplify(se.sympify(T), tolerance=1e-10, rational=False)

def geometric_jacobian(Robot: sp.Matrix, q: sp.Matrix = None) -> sp.Matrix:
    # variable initialization
    z0, p0 = sp.Matrix([0, 0, 1]), sp.Matrix([0, 0, 0, 1])
    q = list(Robot.free_symbols) if q is None else q
    n_joints = len(q)
    T = dkin(Robot)
    pe = T*p0; pe.row_del(-1)
    J = se.zeros(6, n_joints)

    # first iteration
    if Robot[0,0] in q:
        # prismatic joint
        J[:,0] = se.Matrix([z0, se.zeros(3,1)])
    else:
        # revolute joint
        J[:,0] = se.Matrix([z0.cross(pe), z0])

    # the rest of iterations
    for joint in range(1,n_joints):
        A = dkin(Robot[:joint,:])
        R = A[:3,:3]; zi_1 = R*z0
        if Robot[joint, 0] in q: 
            # prismatic joint
            J[:,joint] = se.Matrix([zi_1, se.zeros(3,1)])
        else: 
            # revolute joint
            pi_1 = A*p0; pi_1.row_del(-1)
            J[:,joint] = se.Matrix([zi_1.cross(pe-pi_1), zi_1])
    return sp.nsimplify(se.sympify(J), tolerance=1e-10, rational=False)
