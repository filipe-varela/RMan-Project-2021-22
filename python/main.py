# sympy with symbolic and Matrix for everything related with symbolic
# computing

from sympy import *
from dhc.table import generate_robot
from kinematic.dkin import dkin

def main():
    print("Hello World!")
    Robot, q = generate_robot()
    T = dkin(Robot)
    print(Robot)
    print(T)

if __name__ == "__main__":
    main()