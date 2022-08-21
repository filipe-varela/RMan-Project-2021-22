from sympy import *
import unittest
from kinematic import dkin, geometric_jacobian
from dhc import generate_robot

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
        index = 0
        for element_test, element_real in zip(T_test, T_real):
            if simplify(element_real) != simplify(element_test):
                return f"""

Elements that were diferent at index={index}:
Test: {element_test}
Real: {element_real}
"""
            index += 1

    def test_direct_kinematics(self):
        Robot, q, a = self.table_test()
        T_test = dkin(Robot)
        T_real = Matrix([[cos(q[0])*cos(q[1]),  sin(q[0]), cos(q[0])*sin(q[1]), q[2]*cos(q[0])*sin(q[1]) + a[1]*cos(q[0])*cos(q[1])],
                         [sin(q[0])*cos(q[1]), -cos(q[0]), sin(q[0])*sin(q[1]), q[2]*sin(q[0])*sin(q[1]) + a[1]*sin(q[0])*cos(q[1])],
                         [          sin(q[1]),          0,          -cos(q[1]),                     a[1]*sin(q[1]) - q[2]*cos(q[1])],
                         [                  0,          0,                   0,                                                   1]])
        self.assertEqual(
            simplify(T_test - T_real),
            zeros(4), 
            self.error_test_generating_kinematics_str(T_test, T_real)
        )
    
    def test_geometric_jacobian(self):
        Robot, q, a = self.table_test()
        J_test = geometric_jacobian(Robot, q=q)
        J_real = simplify(
            Matrix(
                [                
                    [-q[2]*sin(q[0])*sin(q[1])-a[1]*sin(q[0])*cos(q[1]),                                                                                 -cos(q[0])*(-q[2]*cos(q[1])+a[1]*sin(q[1])),   cos(q[0])*sin(q[1])],
                    [ q[2]*cos(q[0])*sin(q[1])+a[1]*cos(q[0])*cos(q[1]),                                                                                   sin(q[0])*(q[2]*cos(q[1])-a[1]*sin(q[1])),   sin(q[0])*sin(q[1])],
                    [                                                 0, sin(q[0])*(q[2]*sin(q[0])*sin(q[1])+a[1]*sin(q[0])*cos(q[1]))+cos(q[0])*(q[2]*cos(q[0])*sin(q[1])+a[1]*cos(q[0])*cos(q[1])),            -cos(q[1])],
                    [                                                 0,                                                                                                                   sin(q[0]),                     0],
                    [                                                 0,                                                                                                                  -cos(q[0]),                     0],
                    [                                                 1,                                                                                                                          0,                      0]
                ]
            )
        )
        self.assertEqual(
            J_real,
            J_test,
            self.error_test_generating_kinematics_str(
                J_test,
                J_real
            )
        )

    def initialize_kinematic_2(self):
        Robot, q = generate_robot()
        T = dkin(Robot)
        pe_real = Matrix([0,0,.213+.25+.26+.3385+.1385]).evalf(chop=1e-2)
        R_real  = Matrix([[0, -1, 0],
                          [1,  0, 0],
                          [0,  0, 1]])
        return q, T, pe_real, R_real

    def test_kinematic_2_position(self):
        q, T, pe_real, _ = self.initialize_kinematic_2()
        pe_test = Matrix(
            T.evalf(subs=dict(zip(q, zeros(1,7))))
             .evalf(chop=True)[:3,-1]
        )
        self.assertEqual(
            pe_test.evalf(2),
            pe_real.evalf(2), 
            msg=self.error_test_generating_kinematics_str(
                pe_test, 
                pe_real
            )
        )

    def test_kinematic_2_orientation(self):
        q, T, _, R_real = self.initialize_kinematic_2()
        R_test = Matrix(
            T.evalf(subs=dict(zip(q, zeros(1,7))))
             .evalf(chop=True)[:3,:3]
        ).evalf(2,chop=True)
        self.assertEqual(
            R_test,
            R_real,
            self.error_test_generating_kinematics_str(
                R_test,
                R_real
            )
        )

if __name__ == "__main__": 
    unittest.main()