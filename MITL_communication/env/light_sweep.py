"""
Author Wei Wang (wei7@kth.se)

This file is used to generate the SWEEP trajectory for agile light-duty robot.
"""

import copy
import numpy as np
import math
from light_model import AgentEnv
from light_jps import step_path, jump_point_search


class Sweep:

    def __init__(self, map_ori, map_shape, agent_light, agent_heavy, h_a):
        self.map = map_ori
        self.map_row_s, self.map_row_e, self.map_col_s, self.map_col_e = map_shape
        self.agent_l = agent_light
        self.agent_h = agent_heavy
        self.h_a_init = h_a
        self.h_a = h_a
        self.start_list = [np.array([self.map_row_s + self.agent_l.r_sens,     self.map_col_s]),
                           np.array([self.map_row_s + self.agent_l.r_sens,     self.map_col_e - 1]),
                           np.array([self.map_row_e - self.agent_l.r_sens - 1, self.map_col_s]),
                           np.array([self.map_row_e - self.agent_l.r_sens - 1, self.map_col_e - 1])]

    def closest_node(self):
        node = self.agent_l.p_pos
        nodes = np.asarray(self.start_list)
        dist_2 = np.sum(np.abs((nodes - node)), axis=1)
        return np.argmin(dist_2)

    def get_ready_next_sweep(self):
        # find the closest starting position of the sweep path
        start_node_i = self.closest_node()
        # print("Possible starting locations are: ", self.start_list)
        sweep_start = self.start_list[start_node_i]
        print("The closest start location is ", sweep_start)
        path_ready_to_sweep = step_path(jump_point_search(self.map, tuple(sweep_start), tuple(self.agent_l.p_pos)), self.map)
        return start_node_i, sweep_start, path_ready_to_sweep

    def sweep_path_random(self):
        # find the closest starting position of the sweep path
        start_node_i, sweep_start, path_ready_to_sweep = self.get_ready_next_sweep()
        path = [np.array(i) for i in path_ready_to_sweep]

        # length of the parallel line
        len_central = self.map_col_e - self.map_col_s

        # k=1: start from left to right, k=-1: start from right to left
        if start_node_i % 2 == 0:
            k = 1
        if start_node_i % 2 == 1:
            k = -1

        # initial the sweep starting position and the sweep path
        self.agent_l.p_pos = sweep_start
        path.append(np.array(sweep_start))

        # sweep from top to bottom
        if start_node_i < 2:
            while self.map_row_s <= self.agent_l.p_pos[0] < self.map_row_e:
                for i in range(len_central - 1):
                    self.agent_l.p_pos[1] = self.agent_l.p_pos[1] + k
                    # print("11_Current position: ", self.agent_l.p_pos)
                    path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))

        # when the hatch distance does not need to adjust
                if self.agent_l.p_pos[0] + self.agent_l.r_sens >= self.map_row_e-1:
                    # print("1_break")
                    break
                elif self.agent_l.p_pos[0] + self.h_a < self.map_row_e:
                    for j in range(self.h_a - 1):
                        self.agent_l.p_pos[0] = self.agent_l.p_pos[0] + 1
                        # print("12_Current position: ", self.agent_l.p_pos)
                        path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))

        # when the hatch distance needs to adjust
                else:
                    while self.agent_l.p_pos[0] + self.h_a >= self.map_row_e and self.h_a > self.agent_l.r_sens + 1:
                        self.h_a = self.h_a - 1
                    for j in range(self.h_a - 1):
                        self.agent_l.p_pos[0] = self.agent_l.p_pos[0] + 1
                        # print("13_Current position: ", self.agent_l.p_pos)
                        path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))

                self.agent_l.p_pos[0] = self.agent_l.p_pos[0] + 1
                # print("14_Current position: ", self.agent_l.p_pos)
                path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))
                k = -k

        # sweep from bottom to top
        else:
            while self.map_row_s <= self.agent_l.p_pos[0] < self.map_row_e:
                # print("2_", self.map_row_s, self.agent_l.p_pos[0], self.map_row_e)
                for i in range(len_central - 1):
                    self.agent_l.p_pos[1] = self.agent_l.p_pos[1] + k
                    # print("21_Current position: ", self.agent_l.p_pos)
                    path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))

        # when the hatch distance does not need to adjust
                if self.agent_l.p_pos[0] - self.agent_l.r_sens <= 0:
                    # print("2_break")
                    break
                elif self.agent_l.p_pos[0] - self.h_a >= 0:
                    for j in range(self.h_a - 1):
                        self.agent_l.p_pos[0] = self.agent_l.p_pos[0] - 1
                        # print("22_Current position: ", self.agent_l.p_pos)
                        path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))

        # when the hatch distance needs to adjust
                else:
                    while self.agent_l.p_pos[0] - self.h_a < 0 and self.h_a > self.agent_l.r_sens + 1:
                        self.h_a = self.h_a - 1
                    for j in range(self.h_a - 1):
                        self.agent_l.p_pos[0] = self.agent_l.p_pos[0] - 1
                        # print("23_Current position: ", self.agent_l.p_pos)
                        path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))
                self.agent_l.p_pos[0] = self.agent_l.p_pos[0] - 1
                # print("24_Current position: ", self.agent_l.p_pos)
                path.append(np.array([self.agent_l.p_pos[0], self.agent_l.p_pos[1]]))
                k = -k

        return path

    def detection_simple(self, path, v_light):
        k = 0
        while k < len(path):
            if (abs(path[k][0] - path[k][2]) >= (self.agent_l.r_sens + self.agent_h.r_sens) or
                abs(path[k][1] - path[k][3]) >  (self.agent_l.r_sens + self.agent_h.r_sens)) \
                    or (abs(path[k][0] - path[k][2]) >  (self.agent_l.r_sens + self.agent_h.r_sens) or
                        abs(path[k][1] - path[k][3]) >= (self.agent_l.r_sens + self.agent_h.r_sens)):
                k += 1
                if k % v_light == 1:
                    print("Step : ", math.floor((k - 1) / v_light) + 1, "Heavy: ", [path[k][2], path[k][3]], "Light: ",
                          [path[k][0], path[k][1]])
            else:
                print("The heavy duty agent is detected at step: ", math.floor((k - 1) / v_light) + 1, ". Pos_heavy: ",
                      [path[k][2], path[k][3]],
                      " Pos_light: ", [path[k][0], path[k][1]])
                break
        return k


def detection(world, map_list, map_row, map_col, num_round_exchange, h_a, r_sens_heavy, r_sens_light, figfolder):
    # start sweeping

    num_agent_heavy = len(map_list)
    path_sweep = [[np.array([0, 0])] for i in range(num_agent_heavy)]

    while world.agents_light.num_round < num_round_exchange:  # <=
        map = np.zeros((map_row, map_col), dtype=int)
        for i in range(num_agent_heavy):
            print("Round ", world.agents_light.num_round, " Region", (i + 1))
            world.agents_light.region = i

            # obtain the light-duty agent sweep path
            path_sweep[i] = Sweep(map, map_list[i], world.agents_light, world.agents_heavy[i],
                                  h_a).sweep_path_random()
            map_sweep = copy.deepcopy(map)

            # start to search heavy-duty agents
            for j in range(len(path_sweep[i])):
                if (abs(path_sweep[i][j][0] - world.agents_heavy[i].p_pos[0]) > (r_sens_light + r_sens_heavy) or
                    abs(path_sweep[i][j][1] - world.agents_heavy[i].p_pos[1]) > (r_sens_light + r_sens_heavy)) \
                        or (abs(path_sweep[i][j][0] - world.agents_heavy[i].p_pos[0]) > (r_sens_light + r_sens_heavy) or
                            abs(path_sweep[i][j][1] - world.agents_heavy[i].p_pos[1]) > (r_sens_light + r_sens_heavy)):
                    if len(world.agents_heavy[i].path) % world.agents_light.velocity == 0:
                        # obtain the heavy-duty agent path
                        for k in range(num_agent_heavy):
                            world.agents_heavy[k].p_pos = AgentEnv(map_list[k], world.agents_heavy[k].p_pos).move()
                        print("Step : ", math.floor(len(world.agents_heavy[i].path) / world.agents_light.velocity) + 1,
                              "Heavy: ", [world.agents_heavy[i].p_pos[0], world.agents_heavy[i].p_pos[1]],
                              "Light: ", [path_sweep[i][j][0], path_sweep[i][j][1]],
                              "Distance is: ", abs(world.agents_heavy[i].p_pos[0] - path_sweep[i][j][0]) +
                              abs(world.agents_heavy[i].p_pos[1] - path_sweep[i][j][1]))
                    world.agents_light.p_pos = path_sweep[i][j]
                    world.agents_light.path.append(world.agents_light.p_pos)
                    for kk in range(num_agent_heavy):
                        world.agents_heavy[kk].path.append(np.array(world.agents_heavy[kk].p_pos))
                    # j += 1

                # find heavy-duty agents
                else:
                    """
                    Detection: the sensing area overlaps > 1 time slot
                    """
                    world.agents_light.p_pos = path_sweep[i][j]
                    world.agents_light.path.append(world.agents_light.p_pos)
                    for kk in range(num_agent_heavy):
                        world.agents_heavy[kk].path.append(np.array(world.agents_heavy[kk].p_pos))
                    print("The heavy duty agent is detected at step: ",
                          math.floor(len(world.agents_heavy[i].path) / world.agents_light.velocity) + 1,
                          "Pos_heavy: ", [world.agents_heavy[i].p_pos[0], world.agents_heavy[i].p_pos[1]],
                          "Pos_light: ", [path_sweep[i][j][0], path_sweep[i][j][1]],
                          "Distance is: ", abs(world.agents_heavy[i].p_pos[0] - path_sweep[i][j][0]) +
                          abs(world.agents_heavy[i].p_pos[1] - path_sweep[i][j][1]))
                    break
            print("Stop sweep at Region", (i + 1), " at position ", world.agents_light.p_pos)

            # go to next region
            """
            When reach to the new region, reset cost 1 time slot.
            """
            if i == num_agent_heavy - 1:
                _, _, path_ready_to_sweep = Sweep(map, map_list[0], world.agents_light,
                                                  world.agents_heavy[i], h_a).get_ready_next_sweep()
            else:
                _, _, path_ready_to_sweep = Sweep(map, map_list[i + 1], world.agents_light,
                                                  world.agents_heavy[i], h_a).get_ready_next_sweep()

            for l in range(len(path_ready_to_sweep)):
                if len(world.agents_heavy[i].path) % world.agents_light.velocity == 0:
                    # obtain the heavy-duty agent path
                    for k in range(num_agent_heavy):
                        world.agents_heavy[k].p_pos = AgentEnv(map_list[k], world.agents_heavy[k].p_pos).move()
                world.agents_light.p_pos = path_ready_to_sweep[l]
                world.agents_light.path.append(world.agents_light.p_pos)
                for kk in range(num_agent_heavy):
                    world.agents_heavy[kk].path.append(np.array(world.agents_heavy[kk].p_pos))
        path_detect = world.agents_light.path
        for i in range(num_agent_heavy):
            path_detect = np.concatenate((path_detect, world.agents_heavy[i].path),
                                         axis=1)
        world.agents_light.num_round += 1

    return path_detect
