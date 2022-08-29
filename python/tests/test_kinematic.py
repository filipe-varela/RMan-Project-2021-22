import numpy as np
import unittest
from kinematic import dkin, geometric_jacobian
from dhc import generate_robot
class TestKinematic(unittest.TestCase):
    def table_test(self):
        from math import pi
        from sympy import symbols, Matrix

        q = symbols('q1:4')
        a = symbols('a1:4')

        Robot = Matrix([[0,     q[0],    0, pi/2, 0],
                        [0,     q[1], a[1], pi/2, 0],
                        [q[2],     0,    0,    0, 0]])

        return Robot, q, a

    def error_test_generating_kinematics_str(self, T_test, T_real) -> str:
        from symengine import sympify
        index = 0
        for element_test, element_real in zip(T_test, T_real):
            if sympify(element_real) != sympify(element_test):
                return f"""

Elements that were diferent at index={index}:
Test: {element_test}
Real: {element_real}
"""
            index += 1

    def test_direct_kinematics(self):
        from symengine import sympify, Matrix, cos, sin
        Robot, q, a = self.table_test()
        T_test = dkin(Robot)
        T_real = sympify(Matrix([[cos(q[0])*cos(q[1]),  sin(q[0]), cos(q[0])*sin(q[1]), q[2]*cos(q[0])*sin(q[1]) + a[1]*cos(q[0])*cos(q[1])],
                                  [sin(q[0])*cos(q[1]), -cos(q[0]), sin(q[0])*sin(q[1]), q[2]*sin(q[0])*sin(q[1]) + a[1]*sin(q[0])*cos(q[1])],
                                  [          sin(q[1]),          0,          -cos(q[1]),                     a[1]*sin(q[1]) - q[2]*cos(q[1])],
                                  [                  0,          0,                   0,                                                   1]]))
        self.assertEqual(
            T_test,
            T_real,
            msg = self.error_test_generating_kinematics_str(
                T_test, 
                T_real
            )
        )
    
    def test_geometric_jacobian(self):
        import symengine as se
        import sympy as sp
        from sympy import cos, sin
        Robot, q, a = self.table_test()
        J_test = geometric_jacobian(Robot, q=q)
        J_real = sp.nsimplify(se.sympify(
            se.Matrix(
                [                
                    [-q[2]*sin(q[0])*sin(q[1])-a[1]*sin(q[0])*cos(q[1]),                                                                                 -cos(q[0])*(-q[2]*cos(q[1])+a[1]*sin(q[1])),   cos(q[0])*sin(q[1])],
                    [ q[2]*cos(q[0])*sin(q[1])+a[1]*cos(q[0])*cos(q[1]),                                                                                 -sin(q[0])*(-q[2]*cos(q[1])+a[1]*sin(q[1])),   sin(q[0])*sin(q[1])],
                    [                                                 0, sin(q[0])*(q[2]*sin(q[0])*sin(q[1])+a[1]*sin(q[0])*cos(q[1]))+cos(q[0])*(q[2]*cos(q[0])*sin(q[1])+a[1]*cos(q[0])*cos(q[1])),            -cos(q[1])],
                    [                                                 0,                                                                                                                   sin(q[0]),                     0],
                    [                                                 0,                                                                                                                  -cos(q[0]),                     0],
                    [                                                 1,                                                                                                                          0,                      0]
                ]
            )
        ), tolerance=1e-10, rational=False)
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
        pe_real = np.array([0,0,.213+.25+.26+.3385+.1385])
        R_real  = np.array([[0, -1, 0],
                            [1,  0, 0],
                            [0,  0, 1]])
        return q, T, pe_real, R_real

    def test_kinematic_2_position(self):
        from sympy import lambdify
        q, T, pe_real, _ = self.initialize_kinematic_2()
        pe_test = np.reshape(lambdify(
            q,
            T[:3,-1],
            modules='numpy'
        )(*[0 for _ in range(len(q))]), (3,))
        pe_test[pe_test < 1e-10] = 0.0
        np.testing.assert_almost_equal(
            pe_test,
            pe_real, 
            decimal = 2,
            err_msg = f"Test: {pe_test}\n\nReal: {pe_real}"
        )

    def test_kinematic_2_orientation(self):
        from sympy import lambdify
        q, T, _, R_real = self.initialize_kinematic_2()
        R_test = lambdify(
            q,
            T[:3,:3],
            modules='numpy'
        )(*[0 for _ in range(len(q))])
        np.testing.assert_almost_equal(
            R_test,
            R_real,
            decimal = 5,
            err_msg = f"Test:{R_test}\n\nReal:{R_real}"
        )

    def test_geometric_jacobian_3_validation(self): pass

    def test_geometric_jacobian_3_singularities_arm(self): pass

    def test_geometric_jacobian_3_singularities_wrist(self): pass

if __name__ == "__main__": 
    unittest.main()