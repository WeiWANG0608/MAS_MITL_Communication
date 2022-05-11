"""
Author Wei Wang (wei7@kth.se)

This file is used to monitor and execute the request exchange process
- task_monitor(): monitors the task completion at each step
- collect_deliver(): requests collection and delivery by light-duty robot and
                     timed runs synthesis after receiving requests
"""

import numpy as np
import copy
import math
from light_sweep import Sweep
from heavy_path_generation import get_update_path


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


def task_monitor(world, states_work_region, v_light):
    # check what tasks have been done if no request has accepted
    for l in range(len(world.agents_heavy)):
        task_check = copy.deepcopy(world.agents_heavy[l].task_left)
        path_check = [list(item) for item in world.agents_heavy[l].path]
        if len(task_check) > 0 and len(world.agents_heavy[l].path) > 1:
            for ll in range(len(task_check)):
                task_co = states_work_region[l][task_check[ll][3]]
                if task_check[ll][0] == 'E' and (task_co == list(world.agents_heavy[l].p_pos) or
                                                 task_co in path_check):
                    world.agents_heavy[l].task_left.remove(task_check[ll])
                    print(f"{bcolors.OKCYAN}task ", task_check[ll], "has been done by robot ", l)
                    print(f"{bcolors.OKCYAN}The path in the past is ", path_check)
                    print(f"{bcolors.OKCYAN}The tasks left ", world.agents_heavy[l].task_left)

                if task_check[ll][0] == 'G' and path_check.count(task_co) == (int(
                        task_check[ll][2]) - int(task_check[ll][1]) + 1) * v_light:
                    world.agents_heavy[l].task_left.remove(task_check[ll])
                    print(f"{bcolors.OKCYAN}task ", task_check[ll], "has been done by robot ", l)
                    print(f"{bcolors.OKCYAN}The path in the past is ", path_check)
                    print(f"{bcolors.OKCYAN}The tasks left ", world.agents_heavy[l].task_left)

        return world


def collect_deliver(world, map_obs_work, num_round_exchange, num_agent_heavy, work_region, states_work_region,
                    independent_path_2d, h_a, r_0, v_l, file_folder):
    map_row, map_col = map_obs_work.shape

    record_working_region = [[] for i in range(num_agent_heavy)]
    record_task_left = [[] for i in range(num_agent_heavy)]
    path_sweep = [[np.array([0, 0])] for i in range(num_agent_heavy)]
    path_detect_round = [[] for i in range(num_round_exchange)]

    while world.agents_light.num_round < num_round_exchange:
        map_light = np.zeros((map_row, map_col), dtype=int)
        i = 0
        ii = 0
        for k in range(num_agent_heavy):
            world.agents_heavy[k].detected = False

        for i in range(num_agent_heavy):
            print(f"{bcolors.ENDC}Round ", world.agents_light.num_round, " Region", i)
            world.agents_light.region = i

            # obtain the light-duty agent sweep path
            path_sweep[i] = Sweep(map_light, work_region[i], world.agents_light, world.agents_heavy[i],
                                  h_a).sweep_path_random()

            # start to search heavy-duty agents
            for j in range(len(path_sweep[i])):
                print(f"{bcolors.OKGREEN}Step ",
                      round(len(world.agents_heavy[i].path) / world.agents_light.velocity),
                      "Robot", i, "Pos", world.agents_heavy[i].p_pos, "R0 pos: ", world.agents_light.p_pos)

                # check what tasks have been done if no request has accepted
                world = task_monitor(world, states_work_region, v_l)

                # meet heavy-duty agents
                if (abs(path_sweep[i][j][0] - world.agents_heavy[i].p_pos[0]) <= r_0 and
                        abs(path_sweep[i][j][1] - world.agents_heavy[i].p_pos[1]) <= r_0 and
                        not world.agents_heavy[i].detected):

                    # # """
                    # # Detection: the sensing area overlaps > 1 time slot
                    # # """
                    world.agents_light.p_pos = path_sweep[i][j]
                    world.agents_light.path.append(world.agents_light.p_pos)
                    for kk in range(num_agent_heavy):
                        world.agents_heavy[kk].path.append(np.array(world.agents_heavy[kk].p_pos))
                        record_working_region[kk].append(str(world.agents_heavy[kk].working_region))
                        tl = [world.agents_heavy[kk].task_left[ll][3] for ll in
                              range(len(world.agents_heavy[kk].task_left))]
                        record_task_left[kk].append(tl)

                    # # # when the R_L and R_Hi meet, need to exchange information: - give the request to R_L and
                    # # #                                                           - check if there is a request for them

                    if any(world.agents_heavy[i].request_send):
                        for kk in range(num_agent_heavy):
                            # R_0 collects requests if exist:
                            if any(world.agents_heavy[i].request_send[kk]):
                                target_RH = kk
                                print("len()", len(world.agents_heavy[i].request_send[kk]))
                                for kl in range(len(world.agents_heavy[i].request_send[kk])):
                                    print("kl: ", kl)
                                    if len(world.agents_heavy[i].request_send[kk][kl]) > 0:
                                        world.agents_heavy[i].request_send[kk][kl][4] = str(i)
                                        world.agents_light.request_list[target_RH].append(
                                            world.agents_heavy[i].request_send[kk][kl])
                                    world.agents_heavy[i].request_send[kk] = []

                    # R_0 delivers requests at second round
                    if world.agents_light.num_round > 0 and len(world.agents_light.request_list[i]) > 0:
                        for kl in range(len(world.agents_light.request_list[i])):
                            if int(world.agents_light.request_list[i][kl][4]) not in world.agents_heavy[i].region2go:
                                world.agents_heavy[i].region2go.append(
                                    int(world.agents_light.request_list[i][kl][4]))
                            world.agents_heavy[i].request_receive.append(world.agents_light.request_list[i][kl])

                        current_pos = world.agents_heavy[i].path[-1]
                        current_time = round(len(world.agents_heavy[i].path) / v_l)
                        new_path, new_spec, state_dict_new = get_update_path(current_pos, current_time,
                                                                             world.agents_heavy[i].region2go,
                                                                             world.agents_heavy[i].request_receive,
                                                                             i, world.agents_heavy[i].task_left,
                                                                             states_work_region,
                                                                             map_obs_work, work_region, file_folder)
                        states_work_region[i] = state_dict_new
                        for wr in range(len(world.agents_heavy[i].region2go)):
                            world.agents_heavy[i].working_region = str(world.agents_heavy[i].working_region) + ',' + \
                                                                   str(world.agents_heavy[i].region2go[wr])
                        world.agents_heavy[i].task_left = new_spec
                        # if there is a new path, then the path updated
                        if len(new_path) > 0:
                            print(f"{bcolors.ENDC}The robot ", i, " is updating the path since time ",
                                  round(len(world.agents_heavy[i].path) / v_l))
                            independent_path_2d[i] = independent_path_2d[i][
                                                     0: round(len(world.agents_heavy[i].path) / v_l)]
                            independent_path_2d[i] = independent_path_2d[i] + new_path

                        world.agents_heavy[i].request_receive = []
                        world.agents_heavy[i].region2go = []
                        world.agents_light.request_list[i] = []

                    print(f"{bcolors.WARNING}The heavy duty agent ", i, " is detected at step: ",
                          math.floor(len(world.agents_heavy[i].path) / world.agents_light.velocity),
                          ". Pos_heavy: ", [world.agents_heavy[i].p_pos[0], world.agents_heavy[i].p_pos[1]],
                          ". Pos_light: ", [path_sweep[i][j][0], path_sweep[i][j][1]],
                          ". Distance is: ", abs(world.agents_heavy[i].p_pos[0] - path_sweep[i][j][0]) +
                          abs(world.agents_heavy[i].p_pos[1] - path_sweep[i][j][1]))
                    world.agents_heavy[i].detected = True
                    i += 1
                    break

                else:
                    # if R_0 did not meet R_i in region i yet
                    if len(world.agents_heavy[i].path) % world.agents_light.velocity == 0:
                        # R_H move one step every |v_L| step R_L moves
                        for k in range(num_agent_heavy):
                            # when the heavy ones still finishing local task
                            if len(independent_path_2d[k]) >= len(world.agents_heavy[i].path) / v_l + 1:
                                world.agents_heavy[k].p_pos = independent_path_2d[k][
                                    round(len(world.agents_heavy[i].path) / v_l)]
                            else:
                                world.agents_heavy[k].p_pos = independent_path_2d[k][-1]
                            tl = [world.agents_heavy[k].task_left[ll][3] for ll in
                                  range(len(world.agents_heavy[k].task_left))]
                            print(f"{bcolors.OKBLUE}Step ",
                                  round(len(world.agents_heavy[k].path) / world.agents_light.velocity),
                                  "Robot", k, "Task left: ", tl)

                        print(f"{bcolors.WARNING}Step : ",
                              math.floor(len(world.agents_heavy[i].path) / world.agents_light.velocity),
                              "Light: ", [path_sweep[i][j][0], path_sweep[i][j][1]],
                              "Heavy", i, ": ", [world.agents_heavy[i].p_pos[0], world.agents_heavy[i].p_pos[1]],
                              "Distance is: ", abs(world.agents_heavy[i].p_pos[0] - path_sweep[i][j][0]) +
                              abs(world.agents_heavy[i].p_pos[1] - path_sweep[i][j][1]))


                    world.agents_light.p_pos = path_sweep[i][j]
                    world.agents_light.path.append(world.agents_light.p_pos)
                    for kk in range(num_agent_heavy):
                        world.agents_heavy[kk].path.append(np.array(world.agents_heavy[kk].p_pos))
                        record_working_region[kk].append(str(world.agents_heavy[kk].working_region))
                        tl = [world.agents_heavy[kk].task_left[ll][3] for ll in
                              range(len(world.agents_heavy[kk].task_left))]
                        record_task_left[kk].append(tl)


            print(f"{bcolors.ENDC}Stop sweep at Region", i, " at position ", world.agents_light.p_pos)

            # go to next region
            """
            When reach to the new region, reset cost 1 time slot.
            """

            ii = i

            if ii == num_agent_heavy:
                ii = 0
                _, _, path_ready_to_sweep = Sweep(map_light, work_region[ii], world.agents_light,
                                                  world.agents_heavy[ii], h_a).get_ready_next_sweep()
            else:
                _, _, path_ready_to_sweep = Sweep(map_light, work_region[ii], world.agents_light,
                                                  world.agents_heavy[ii - 1], h_a).get_ready_next_sweep()
            world.agents_heavy[ii].detected = False
            # check if meet next heavy robot during the region swish
            for j in range(len(path_ready_to_sweep)):

                # check what tasks have been done if no request has accepted
                world = task_monitor(world, states_work_region, v_l)

                # meet heavy-duty agents
                if (abs(path_ready_to_sweep[j][0] - world.agents_heavy[ii].p_pos[0]) <= r_0 and
                        abs(path_ready_to_sweep[j][1] - world.agents_heavy[ii].p_pos[1]) <= r_0 and
                        not world.agents_heavy[ii].detected):

                    # # """
                    # # Detection: the sensing area overlaps > 1 time slot
                    # # """
                    world.agents_light.p_pos = np.array(path_ready_to_sweep[j])
                    world.agents_light.path.append(world.agents_light.p_pos)
                    for kk in range(num_agent_heavy):
                        world.agents_heavy[kk].path.append(np.array(world.agents_heavy[kk].p_pos))
                        record_working_region[kk].append(str(world.agents_heavy[kk].working_region))
                        tl = [world.agents_heavy[kk].task_left[ll][3] for ll in
                              range(len(world.agents_heavy[kk].task_left))]
                        record_task_left[kk].append(tl)

                    # # # when the R_L and R_Hi meet, need to exchange information: - give the request to R_L and
                    # # #                                                           - check if there is a request for them

                    if any(world.agents_heavy[ii].request_send):
                        for kk in range(num_agent_heavy):
                            # R_0 collects requests if exist:
                            if any(world.agents_heavy[ii].request_send[kk]):
                                target_RH = kk
                                for kl in range(len(world.agents_heavy[ii].request_send[kk])):
                                    if len(world.agents_heavy[ii].request_send[kk][kl]) > 0:
                                        world.agents_heavy[ii].request_send[kk][kl][4] = str(i)
                                        world.agents_light.request_list[target_RH].append(
                                            world.agents_heavy[ii].request_send[kk][kl])
                                    world.agents_heavy[ii].request_send[kk] = []

                    # R_0 delivers requests at second round
                    if world.agents_light.num_round > 0 and len(world.agents_light.request_list[ii]) > 0:
                        for kl in range(len(world.agents_light.request_list[ii])):
                            if int(world.agents_light.request_list[ii][kl][4]) not in world.agents_heavy[ii].region2go:
                                world.agents_heavy[ii].region2go.append(
                                    int(world.agents_light.request_list[ii][kl][4]))

                            world.agents_heavy[ii].request_receive.append(world.agents_light.request_list[ii][kl])

                        current_pos = world.agents_heavy[ii].path[-1]
                        current_time = round(len(world.agents_heavy[ii].path) / v_l)
                        new_path, new_spec, state_dict_new = get_update_path(current_pos, current_time,
                                                                             world.agents_heavy[ii].region2go,
                                                                             world.agents_heavy[ii].request_receive,
                                                                             i, world.agents_heavy[ii].task_left,
                                                                             states_work_region,
                                                                             map_obs_work, work_region, file_folder)
                        states_work_region[ii] = state_dict_new
                        for wr in range(len(world.agents_heavy[ii].region2go)):
                            world.agents_heavy[ii].working_region = str(world.agents_heavy[ii].working_region) + ',' + \
                                                                   str(world.agents_heavy[ii].region2go[wr])
                        world.agents_heavy[ii].task_left = new_spec
                        # if there is a new path, then the path updated
                        if len(new_path) > 0:
                            print(f"{bcolors.ENDC}The robot ", i, " is updating the path since time ",
                                  round(len(world.agents_heavy[ii].path) / v_l))
                            independent_path_2d[ii] = independent_path_2d[ii][
                                                      0: round(len(world.agents_heavy[ii].path) / v_l)]
                            independent_path_2d[ii] = independent_path_2d[ii] + new_path

                        world.agents_heavy[ii].request_receive = []
                        world.agents_heavy[ii].region2go = []
                        world.agents_light.request_list[ii] = []

                    print("i: ", i, " ii: ", ii, "j: ", j, 'len(path_sweep)', len(path_sweep), 'len(path_sweep[ii])', len(path_sweep[ii]))
                    print(f"{bcolors.WARNING}The heavy duty agent ", ii, " is detected at step: ",
                          math.floor(len(world.agents_heavy[ii].path) / world.agents_light.velocity),
                          ". Pos_heavy: ", [world.agents_heavy[ii].p_pos[0], world.agents_heavy[ii].p_pos[1]],
                          ". Pos_light: ", [path_sweep[ii][-1][0], path_sweep[ii][-1][1]],
                          ". Distance is: ", abs(world.agents_heavy[ii].p_pos[0] - path_sweep[ii][-1][0]) +
                          abs(world.agents_heavy[ii].p_pos[1] - path_sweep[ii][-1][1]))
                    world.agents_heavy[ii].detected = True
                    i += 1
                    break
                else:
                    # if R_L did not meet R_HI in region i yet
                    if len(world.agents_heavy[ii].path) % world.agents_light.velocity == 0:
                        # R_H move one step every |v_L| step R_L moves
                        for k in range(num_agent_heavy):
                            # when the heavy ones still finishing local task
                            if len(independent_path_2d[k]) >= len(world.agents_heavy[ii].path) / v_l + 1:
                                world.agents_heavy[k].p_pos = independent_path_2d[k][
                                    round(len(world.agents_heavy[ii].path) / v_l)]
                            else:
                                world.agents_heavy[k].p_pos = independent_path_2d[k][-1]

                            tl = [world.agents_heavy[k].task_left[ll][3] for ll in
                                  range(len(world.agents_heavy[k].task_left))]
                            print(f"{bcolors.OKBLUE}Step ",
                                  round(len(world.agents_heavy[k].path) / world.agents_light.velocity),
                                  "Robot", k, "Task left: ", tl)

                        print(f"{bcolors.WARNING}Step : ",
                              math.floor(len(world.agents_heavy[ii].path) / world.agents_light.velocity),
                              "Light: ", [path_ready_to_sweep[j][0], path_ready_to_sweep[j][1]],
                              "Heavy ", i, ": ", [world.agents_heavy[ii].p_pos[0], world.agents_heavy[ii].p_pos[1]],
                              "Distance is: ", abs(world.agents_heavy[ii].p_pos[0] - path_ready_to_sweep[j][0]) +
                              abs(world.agents_heavy[ii].p_pos[1] - path_ready_to_sweep[j][1]))

                    world.agents_light.p_pos = np.array(path_ready_to_sweep[j])
                    world.agents_light.path.append(world.agents_light.p_pos)
                    for kk in range(num_agent_heavy):
                        world.agents_heavy[kk].path.append(np.array(world.agents_heavy[kk].p_pos))
                        record_working_region[kk].append(str(world.agents_heavy[kk].working_region))
                        tl = [world.agents_heavy[kk].task_left[ll][3] for ll in
                              range(len(world.agents_heavy[kk].task_left))]
                        record_task_left[kk].append(tl)

        path_detect_round[world.agents_light.num_round] = world.agents_light.path

        for j in range(num_agent_heavy):
            path_detect_round[world.agents_light.num_round] = np.concatenate(
                (path_detect_round[world.agents_light.num_round],
                 world.agents_heavy[j].path), axis=1)

        world.agents_light.num_round += 1

    return path_detect_round, independent_path_2d, record_working_region, record_task_left


def complete_path(world, v_light, independent_path_2d, states_work_region, record_working_region, record_task_left):
    num_agent_heavy = len(world.agents_heavy)
    # finish the current time step
    while len(world.agents_heavy[0].path) % v_light != 0:
        for i in range(num_agent_heavy):
            world.agents_heavy[i].path.append(world.agents_heavy[i].path[-1])
        world.agents_light.path.append(world.agents_light.path[-1])

    # finish the new step
    current_step = round(len(world.agents_heavy[0].path) / v_light)
    max_length = max([len(i) for i in independent_path_2d])
    for i in range(current_step, max_length):
        for t in range(v_light):
            world.agents_light.path.append(world.agents_light.path[-1])
            for j in range(num_agent_heavy):
                if i >= len(independent_path_2d[j]):
                    world.agents_heavy[j].path.append(world.agents_heavy[j].path[-1])
                else:
                    world.agents_heavy[j].path.append(independent_path_2d[j][i])

                if len(world.agents_heavy[j].path) % world.agents_light.velocity == 0:
                    print(f"{bcolors.WARNING}Step : ",
                          round(len(world.agents_heavy[j].path) / world.agents_light.velocity))
                    # check what tasks has been done if no request has accepted
                    task_check = copy.deepcopy(world.agents_heavy[j].task_left)
                    path_check = [list(item) for item in world.agents_heavy[j].path]
                    if len(task_check) > 0:
                        for ll in range(len(task_check)):
                            task_co = states_work_region[j][task_check[ll][3]]
                            if task_check[ll][0] == 'E' and (task_co == list(world.agents_heavy[j].p_pos) or
                                                             task_co in path_check):
                                world.agents_heavy[j].task_left.remove(task_check[ll])
                                print(f"{bcolors.OKCYAN}task ", task_check[ll], "has been done by robot ", j)
                                # print(f"{bcolors.OKCYAN}The path in the past is ", path_check)
                                print(f"{bcolors.OKCYAN}The tasks left ", world.agents_heavy[j].task_left)

                            if task_check[ll][0] == 'G' and path_check.count(task_co) == (int(
                                    task_check[ll][2]) - int(task_check[ll][1]) + 1) * v_light:
                                world.agents_heavy[j].task_left.remove(task_check[ll])
                                print(f"{bcolors.OKCYAN}task ", task_check[ll], "has been done by robot ",j)
                                # print(f"{bcolors.OKCYAN}The path in the past is ", path_check)
                                print(f"{bcolors.OKCYAN}The tasks left ", world.agents_heavy[j].task_left)

                record_working_region[j].append(record_working_region[j][-1])
                tl = [world.agents_heavy[j].task_left[n][3] for n in range(len(world.agents_heavy[j].task_left))]
                record_task_left[j].append(tl)
                print(f"{bcolors.OKBLUE}Step ", round(len(world.agents_heavy[j].path) / world.agents_light.velocity),
                      "Robot", j, "Task left: ", tl)

    return independent_path_2d, record_working_region, record_task_left
