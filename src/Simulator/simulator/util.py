import random
import sys


def parse_init_pose(num, poses):
    '''
    This function parses the initial positions of models; if argument 'poses' is an empty list,
    set models to random positions
    :param num: Number of models
    :param poses: User_input list of initial positions
    :return: init_poses - parsed initial positions
    '''
    init_poses = []
    for pose in poses:
        pose = list((int(p) for p in pose.strip('()').split(',')))
        pose.append(0.3)
        init_poses.append(tuple(pose))
    if len(poses) < num:
        for _ in range(num - len(poses)):
            pose = (random.uniform(-10, 10), random.uniform(-10, 10), 0.3)
            init_poses.append(pose)
    return init_poses


def parse_goal_pose(num, poses, model):
    '''
    This function parses the goal positions of models; if argument 'poses' is empty,
    set models' goals to random positions
    :param num: Number of models
    :param poses: User_input list of goal positions
    :param model: the model type
    :return: goal_poses - parsed goal positions
    '''
    if len(poses) > 0 and type(poses[0]) == list:
        return poses
    goal_poses = []
    for pose in poses:
        pose = list((int(p) for p in pose.strip('()').split(',')))
        if model == 'car': pose.append(0)
        goal_poses.append(tuple(pose))
    if len(poses) < num:
        for _ in range(num - len(poses)):
            if model == 'car': pose = (random.uniform(-20, 20), random.uniform(-20, 20), 0)
            elif model == 'drone': pose = (random.uniform(-20, 20), random.uniform(-20, 20), random.uniform(1, 10))
            goal_poses.append(pose)
    return goal_poses


def sim_launch(models, loc):
    '''
    This function calls launch method to create launch file and use roslaunch to initiate simulator
    :param models: A dictionary that contains number of drones and number cars
    :param loc: Initial location of models
    :return: The initiated Gazebo-ROS process
    '''
    import launch
    ros_proc = launch.launch(models, loc)
    print("============= Simulator starts successful ================")
    # time.sleep(max(num * 6, 10))
    return ros_proc
    
if __name__ == '__main__':
    print(parse_goal_pose(2, [], 'car'))