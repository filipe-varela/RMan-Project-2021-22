from sympy import *
import unittest
from kinematic import dkin

init_printing(use_unicode=True)

class TestKinematic(unittest.TestCase):
    def table_test(self):
        from math import pi

        q = symbols('q1:4')
        a = symbols('a1:4')

        Robot = Matrix([[0,     q[0],    0, pi/2, 0],
                        [0,     q[1], a[1], pi/2, 0],
                        [q[2],     0,    0,    0, 0]])

        return Robot, q, a

    def error_test_generating_kinematics_str(self, T_test: Matrix, T_real: Matrix) -> str:
        for element_test, element_real in zip(T_test, T_real):
            if element_real != element_test:
                return f"""

Elements that were diferent:
Test: {element_test}
Real: {element_real}
"""

    def test_generating_kinematics(self):
        Robot, q, a = self.table_test()
        T_test = dkin(Robot)
        T_real = Matrix([[cos(q[0])*cos(q[1]),  sin(q[0]), cos(q[0])*sin(q[1]), q[2]*cos(q[0])*sin(q[1]) + a[1]*cos(q[0])*cos(q[1])],
                         [sin(q[0])*cos(q[1]), -cos(q[0]), sin(q[0])*sin(q[1]), q[2]*sin(q[0])*sin(q[1]) + a[1]*sin(q[0])*cos(q[1])],
                         [          sin(q[1]),          0,          -cos(q[1]),                     a[1]*sin(q[1]) - q[2]*cos(q[1])],
                         [                  0,          0,                   0,                                                   1]])
        
        self.assertTrue(
            nsimplify(T_test - T_real) == zeros(4), 
            self.error_test_generating_kinematics_str(T_test, T_real)
        )

if __name__ == "__main__": 
    unittest.main()