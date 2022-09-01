# sympy with symbolic and Matrix for everything related with symbolic
# computing

from sympy import *
from rman.dhc import generate_robot
from rman.kinematics.direct import dkin

"""
The goal on this main script is to, given an 
end-effector position and orientation, we could
estimate the values of the vector q while using
the inverse kinematics and then, with the 
direct kinematics to check the output computed,
the arm moves to the correspondent position.
"""


def main():
    print("Hello World!")
    Robot, q = generate_robot()
    T = dkin(Robot)
    print(Robot)
    print(T)

if __name__ == "__main__":
    main()