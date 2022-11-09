def main(): 
    import cProfile
    import pstats
    from dhc import generate_robot
    from kinematic import dkin
    import os 

    with cProfile.Profile() as pr:
        Robot, q = generate_robot()
        T = dkin(Robot)

    stats = pstats.Stats(pr)
    stats.sort_stats(pstats.SortKey.TIME)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    stats.dump_stats(filename=os.path.join(dir_path, "generate_robot_and_dkin.prof"))


if __name__ == "__main__":
    main()
