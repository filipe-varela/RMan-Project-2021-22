# sympy with symbolic and Matrix for everything related with symbolic
# computing

from sympy import *
from dhc.table import generate_robot

def main():
    print("Hello World!")
    Robot, q = generate_robot()
    print(Robot)

if __name__ == "__main__":
    main()