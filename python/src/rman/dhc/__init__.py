def generate_robot(kinematic=True):
    """A main function in which makes the generating of the 
    robot more easier, in which by passing a boolean gives 
    gives you the Robot matrix that you desired.
    
    Returns:
        Robot: the matrix with the kinematics properties if 
        true or the dynamics one if false.
    """
    if kinematic:
        from rman.dhc.kinematic import robot_kinematics
        return robot_kinematics()

if __name__ == "__main__":
    Robot, q = generate_robot()