import random
import sys

def parse_init_pose(num, poses):
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


def sim_launch(num, model, init_poses):
    '''
    This function calls Gazebo simulator with the specified model
    :param num: number of models to be spawned
    :param model: the model that to be launched and spawned
    :param init_poses: initial positions of models
    :return: ros_proc - the Gazebo-ROS process that is running
    '''
    sys.path.insert(0, model)
    import launch
    ros_proc = launch.launch(num, init_poses)
    print("============= Simulator starts successful ================")
    # time.sleep(max(num * 6, 10))
    return ros_proc