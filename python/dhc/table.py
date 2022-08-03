def generate_robot():
    """A generating function which creates the DHC table and
    the consecutive symbolic variables used. The matrix has
    as columns the following labels in this specific order:
    d, v, a, alpha, offset

    Returns:
        Robot: a symbolic matrix for the project's robot
        q: the symbolic array of DoF of the Robot
    """
    from sympy import Matrix, symbols
    from math import pi

    q = symbols('q1:8')

    q1, q2, q3, q4, q5, q6, q7 = q

    Robot = Matrix([[0,     q1, 0,   pi/2,  0],
                    [0,     q2, .25, pi/2,  pi/2],
                    [0,     q3, .26, -pi/2, 0],
                    [0,     q4, 0,   pi/2,  -pi/2],
                    [.3385, q5, 0,   pi/2,  pi/2],
                    [0,     q6, 0,   -pi/2, 0],
                    [.1395, q7, 0,   0,     0]])
    
    return Robot, q

if __name__ == "__main__":
    Robot, q = generate_robot()