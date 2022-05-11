from datetime import datetime
import numpy as np
import math
import copy
import os
import random
from initialization import make_world, get_tasks, initial_env, initial_agents, initial_work_region, initial_specification
from visualization import draw_path, draw_map, animate_solution
from heavy_WTS import total_states
from heavy_path_generation import heavy_path, get_work_map, get_path_cell2coord, get_independent_requests_tasks
from request_exchange import collect_deliver, complete_path


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


if __name__ == '__main__':
    dirname = os.path.dirname
    ABPATH = os.path.join(dirname(dirname(__file__)))

    """create the folder for the fig, file, output(video)"""
    now = datetime.today().strftime('%Y-%m-%d')
    fig_folder = ABPATH + '/figures/' + str(now)
    file_folder = ABPATH + '/files/' + str(now)
    output_folder = ABPATH + '/files/' + str(now) + '/output'
    if not (os.path.exists(fig_folder)):
        os.makedirs(fig_folder)
    if not (os.path.exists(file_folder)):
        os.makedirs(file_folder)
    if not (os.path.exists(output_folder)):
        os.makedirs(output_folder)
    """
    Initial the environment
    Get the number robots: num_agent_heavy, num_agent_light
    Get the size of the work region: region_row, region_col
    Get the whole map with obstacles: 2*(N/2) work regions, obs_percentage, obs_list
    Get the map and state space of the env: map_original, map_obstacle, states_original
    Get the map and state space of work regions for each heavy-duty robot: work_region, map_work_region, states_work_region
    Get initial position of all robots: agents_heavy, agents_light
    """
    num_agent_heavy = random.choice([2, 4])
    num_agent_light = 1
    region_row, region_col = [random.randint(10, 15), random.randint(10, 15)]
    map_row, map_col = [region_row * 2, int(region_col * (num_agent_heavy / 2))]
    obs_percentage = 10
    obs_list = initial_env(region_row * 2, region_col * (num_agent_heavy / 2), obs_percentage)
    map_original = np.zeros((map_row, map_col), dtype=int)
    map_obstacle = copy.deepcopy(map_original)
    for i in range(len(obs_list)):
        map_obstacle[obs_list[i][0]][obs_list[i][1]] = 1
    states_original = total_states(map_obstacle)
    work_region = initial_work_region(num_agent_heavy, region_row, region_col)
    map_work_region = [get_work_map(map_obstacle, work_region, i) for i in range(num_agent_heavy)]
    states_work_region = [total_states(map_work_region[i]) for i in range(num_agent_heavy)]

    agents_heavy, [agents_light] = initial_agents(work_region, obs_list, num_agent_heavy, num_agent_light)
    print("Init heavy_duty agents: ", agents_heavy, ". Init light_duty agent: ", agents_light)

    # get the MITL task specification
    specifications = initial_specification(obs_list, agents_heavy, agents_light, work_region, states_work_region)
    for i in range(num_agent_heavy):
        print("MITL Specification of R", i + 1, ": ", specifications[i])


    # """
    # Initial the environment
    # Get the whole map: map_ori, with size [map_row, map_col]
    # Get the task region in the map for each heavy-duty robot based on map_list: map_TR_list
    # """
    # #  initial map
    # num_agent_heavy = 4
    # num_agent_light = 1
    # map_row, map_col = [20, 20]
    # #  initial agents and specifications
    # # agents_heavy, [agents_light] = initial_agents(map_list, num_agent_heavy, num_agent_light)
    # agents_heavy = [np.array([0, 0]), np.array([0, 19]), np.array([19, 19]), np.array([19, 0])]
    # agents_light = [0, 4]
    # print("Init heavy_duty agents: ", agents_heavy, ". Init light_duty agent: ", agents_light)
    # obs = [[1, 16], [2, 0], [2, 14], [2, 15], [2, 18], [2, 18], [3, 0], [3, 7],
    #        [3, 11], [4, 5], [4, 5], [4, 8], [4, 9], [5, 0], [5, 4], [6, 9],
    #        [7, 4], [7, 18], [8, 1], [9, 9], [10, 7], [10, 16], [10, 19], [11, 1],
    #        [11, 17], [11, 19], [12, 2], [12, 7], [12, 16], [13, 17], [13, 18], [13, 19],
    #        [14, 0], [14, 0], [14, 3], [14, 4], [15, 6], [15, 6], [16, 12], [16, 13],
    #        [17, 0], [17, 9], [17, 12], [17, 19], [18, 0], [19, 9], [19, 10], [19, 17]]
    # map_original = np.zeros((map_row, map_col), dtype=int)
    #
    # map_obstacle = copy.deepcopy(map_original)
    # for i in range(len(obs)):
    #     map_obstacle[obs[i][0]][obs[i][1]] = 1
    # states_dict_ori = total_states(map_obstacle)
    # work_region = [[0, 10, 0, 10], [0, 10, 10, 20], [10, 20, 10, 20], [10, 20, 0, 10]]
    # map_work_region = [get_work_map(map_obstacle, work_region, i) for i in range(num_agent_heavy)]
    # states_work_region = [total_states(map_work_region[i]) for i in range(num_agent_heavy)]
    #
    # specifications = [[['G', '6', '10', 'w13'], ['E', '45', '50', 'w15', 'R3', 'meet']],
    #                   [['G', '12', '15', 'w4'], ['G', '30', '35', 'w8'], ['G', '67', '70', 'w15', 'R0', 'col']],
    #                   [['E', '8', '12', 'w43'], ['G', '25', '28', 'w75'], ['G', '60', '65', 'w55', 'R1', 'meet']],
    #                   [['E', '14', '28', 'w57'], ['E', '20', '30', 'w72']]]


    task_type = get_tasks(specifications, states_work_region)
    map_obs_work = copy.deepcopy(map_obstacle)
    draw_map(map_obs_work, task_type, num_agent_heavy, fig_folder)

    """
    --------------- light-duty robot module ---------------
    """
    # initial the parameters of agents
    u_size = 1
    r_i = 1
    r_0 = 2
    # hatch distance
    h_a = 2 * r_0 - 1
    # velocity / transition weight constraints
    v_heavy = 1
    v_light = max([math.ceil(
        ((2 * (work_region[i][3] - work_region[i][2] - 2 * r_0 - u_size) + h_a) *
         v_heavy) / ((2 * r_0) - (h_a - u_size))) for i in range(num_agent_heavy)])

    """
    --------------- Pre-computation ---------------
    """
    path_existence = [[] for i in range(num_agent_heavy)]
    initial_path = [[] for i in range(num_agent_heavy)]
    independent_path = [[] for i in range(num_agent_heavy)]
    independent_task = [[] for i in range(num_agent_heavy)]
    independent_request = [[] for i in range(num_agent_heavy)]
    independent_path_2d = [[] for i in range(num_agent_heavy)]

    for i in range(num_agent_heavy):
        flag = "R" + str(i)
        # check if the initial specifications are satisfied
        initial_path[i] = heavy_path(map_work_region[i], specifications[i], file_folder, flag,
                                     agents_heavy[i]).get_path()
        # check if robot need to cooperate and provide best promise.  (bool) coop, promise
        if len(initial_path[i]) == 0:
            path_existence[i] = False
            print(path_existence[i])
            print("========There is no path satisfying the specification!========")
        else:
            path_existence[i] = True
    if all(item is True for item in path_existence):
        for i in range(num_agent_heavy):
            independent_task, independent_request = get_independent_requests_tasks(specifications, map_work_region,
                                                                                   agents_heavy, file_folder)
            independent_path[i] = heavy_path(map_work_region[i], independent_task[i], file_folder, flag,
                                             agents_heavy[i]).get_path()
            independent_path_2d[i] = get_path_cell2coord(independent_path[i], states_work_region[i])

    print(f"{bcolors.HEADER}======== ***** The Pre-computation Result ****** ========")
    print(f"{bcolors.HEADER}------- The Original Task Specification -------")
    print(specifications)
    print(f"{bcolors.HEADER}------------ If all the paths exist ------------")
    print(path_existence)
    if all(item is True for item in path_existence):
        print(f"{bcolors.HEADER}---- The Independent Task Specification ----")
        print(independent_task)
        print(f"{bcolors.HEADER}--------- The Independent Request ---------")
        print(independent_request)
        # print(f"{bcolors.HEADER}-------The Path Under The Specification--------")
        # print(independent_path)

    info = ''
    for i in range(num_agent_heavy):
        info = info + str(agents_heavy[i])+';' + ','.join(str(j) for j in specifications[i]) + ';' + ','.join(
            str(j) for j in independent_task[i]) + ';' + ','.join(str(j) for j in independent_request[i]) + '&'
    text_file = open(file_folder + "/" + 'initialization_info' + ".txt", "w")
    text_file.write(info)

    # make the world (core.py)
    world = make_world(agents_heavy, agents_light, r_i, r_0, v_heavy, v_light, u_size, independent_task,
                       independent_request)

    """
    Start two round SWEEP
    """
    # start sweeping
    # maximum 2 rounds to guarantee the requests exchange

    num_round_exchange = 2

    path_detect_round, independent_path_2d, record_working_region, record_task_left = \
        collect_deliver(world, map_obs_work, num_round_exchange, num_agent_heavy, work_region, states_work_region,
                        independent_path_2d, h_a, r_0, v_light, file_folder)

    """
    Path of agile light-duty robot
    """
    path_detect_round[1] = path_detect_round[1][len(path_detect_round[0]): len(path_detect_round[1])]
    for n in range(len(path_detect_round)):
        mm = copy.deepcopy(map_original)
        draw_path(mm, path_detect_round[n], fig_folder, "Trajectory of R0 Round " + str(n + 1))

    """
    Get the data ready for plot and video
    """

    independent_path_2d, record_working_region, record_task_left = \
        complete_path(world, v_light, independent_path_2d, states_work_region, record_working_region, record_task_left)

    record_path_all = world.agents_light.path
    for i in range(num_agent_heavy):
        record_path_all = np.concatenate((record_path_all, world.agents_heavy[i].path), axis=1)
        draw_path(map_obs_work, independent_path_2d[i], fig_folder, "Trajectory of heavy-duty robot R" + str(i + 1))

    st = len(record_working_region[0])
    for i in range(st, len(record_path_all)):
        for j in range(num_agent_heavy):
            record_working_region[j].append(record_working_region[j][-1])
            record_task_left[j].append(record_task_left[j][-1])

    # animate_solution(map_obs_work, record_path_all, record_working_region, record_task_left, r_0, r_i, v_light,
    #                  fig_folder, video=True)

