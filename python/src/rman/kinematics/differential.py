from rman.kinematics.direct import dkin
import sympy as sp
import symengine as se

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
