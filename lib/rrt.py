import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
from lib.calculateFK import FK
# import rtree
import cProfile
import pstats
import time

seed = 0
np.random.seed(seed)

def rrt(map, start: np.ndarray, goal: np.ndarray):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # initialize path
    path = []
    map = enlarge_obstacles(map)

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    # RRT
    start_list = [start]
    goal_list = [goal]
    parent_map = {}
    parent_map[goal.tobytes()] = start
    iteration_limit = 1000
    while iteration_limit > 0:
        iteration_limit -= 1
        random_point = get_random_point_in_space(lowerLim=lowerLim, upperLim=upperLim)

        closet_node_to_start_list = get_closet_node_from_list(target_node=random_point, list=start_list)
        is_collide_with_start_list = is_config_collide(point1_config=random_point, point2_config=closet_node_to_start_list, box=map.obstacles)
        if is_collide_with_start_list == False:
            start_list.append(random_point)
            parent_map[random_point.tobytes()] = closet_node_to_start_list

        closet_node_to_goal_list = get_closet_node_from_list(target_node=random_point, list=goal_list)
        is_collide_with_goal_list = is_config_collide(point1_config=random_point, point2_config=closet_node_to_goal_list, box=map.obstacles)
        if is_collide_with_goal_list == False:
            goal_list.append(random_point)
            parent_map[closet_node_to_goal_list.tobytes()] = random_point

        if is_collide_with_start_list == False and is_collide_with_goal_list == False:
            break

    if iteration_limit == 0:
        return np.array([])

    path.append(goal)
    node = goal
    while np.all(node == start) != True:
        path.append(parent_map[node.tobytes()])
        node = parent_map[node.tobytes()]
    path.reverse()
    return np.array(path)

def get_random_point_in_space(lowerLim: np.ndarray, upperLim: np.ndarray) -> np.ndarray:
    point = []
    for i in range(lowerLim.shape[0]):
        point.append(random.uniform(lowerLim[i], upperLim[i]))
    return np.array(point)

def is_config_collide(point1_config: np.ndarray, point2_config: np.ndarray, box, step=0.1) -> bool:
    step_num = np.round(np.max(np.abs(point2_config - point1_config)) / step)
    immediate_configs = np.linspace(point1_config, point2_config, num=int(step_num))
    forward_knimatics = FK()
    for i in range(immediate_configs.shape[0] - 1):
        joint_pose1, T0e1 = forward_knimatics.forward(q=immediate_configs[i])
        joint_pose2, T0e2 = forward_knimatics.forward(q=immediate_configs[i + 1])
        for block in box:
            if np.any(detectCollision(linePt1=joint_pose1, linePt2=joint_pose2, box=block)):
                return True
    return False

def get_closet_node_from_list(target_node: np.ndarray, list: list) -> np.ndarray:
    min_dis = float('inf')
    min_idx = -1
    for i in range(len(list)):
        dis = np.linalg.norm(target_node - list[i], ord=2)
        if  dis < min_dis:
            min_dis = dis
            min_idx = i
    return list[min_idx]

def enlarge_obstacles(map_struct: map, margin=0.0825) -> map:
    if len(map_struct.obstacles) == 0:
        return map_struct
    else:
        map_struct.obstacles[0, 0 : 3] -= margin
        map_struct.obstacles[0, 3 : 6] += margin
        return map_struct
    
def measure_time(map_struct, start, goal, times=10):
    try:
        start_time = time.time()
        for i in range(times):
            path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
        end_time = time.time()
        print("avg time: " + str((end_time - start_time) / times))
    except:
        measure_time(map_struct=map_struct, start=start, goal=goal, times=times)


if __name__ == '__main__':
    map_idx = 2
    map_struct = loadmap("../maps/map" + str(map_idx) + ".txt")
    starts = [np.array([0, -1, 0, -2, 0, 1.57, 0]),
          np.array([0, 0.4, 0, -2.5, 0, 2.7, 0.707]),
          np.array([0, -1, 0, -2, 0, 1.57, 0]),
          np.array([0, 0.4, 0, -2.5, 0, 2.7, 0.707]),
          np.array([0, -1, 0, -2, 0, 1.57, 0])]
    goals = [np.array([-1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7]),
            np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707]),
            np.array([-0.09234, -0.22543,  0.55477, -2.50348,  0.15922 , 2.30462 , 2.70008]),
            np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707]),
            np.array([-1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7])]
    start = starts[map_idx - 1]
    goal = goals[map_idx - 1]

    # path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    # print(path)

    # # run your function with supervision of profiler
    # with cProfile.Profile() as profile:
    #     path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    #     print(path)

    # # analyze the results
    # results = pstats.Stats(profile)
    # results.sort_stats(pstats.SortKey.TIME)
    # results.print_stats()

    measure_time(map_struct=map_struct, start=start, goal=goal, times=10)
