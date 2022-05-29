"""
Author Wei Wang (wei7@kth.se)

This file is used to initialize the environment and agents
- initial_env(): initialize environment with obs_percent% of obstacles.
- initial_work_region(): initialize work region for each heavy-duty robot
- initial_agents(): initialize agents positions randomly
- initial_specification(): initialize task specifications randomly
- get_tasks(): used to distinguish own, collaboration and meet tasks
- make_world(): objectivize each agent as an object
"""


import random
import copy
import numpy as np
from core import World, Agent_heavy, Agent_light


def initial_env(rows, cols, obs_percent):
    num_obs = round(rows * cols * obs_percent / 100)
    obs_list = []
    i = 0
    while i < num_obs:
        ob = [random.randint(0, rows - 1), random.randint(0, cols - 1)]
        if ob not in obs_list:
            obs_list.append(ob)
            i += 1
    return obs_list


def initial_work_region(num_heavy, rows, cols):
    n_col = int(num_heavy / 2)
    work_region = []
    for i in range(n_col):
        work_region.append([0, rows - 1, i * cols, (i + 1) * cols - 1])
    for i in range(n_col, 0, -1):
        work_region.append([rows, 2 * rows - 1, (i - 1) * cols, i * cols - 1])
    return work_region


def initial_agents(regions, obs, num_heavy, num_light):
    heavy_list = []
    light_list = []
    i = 0
    while i < num_heavy:
        h_pos = [random.randint(regions[i][0], regions[i][1] - 1),
                 random.randint(regions[i][2], regions[i][3] - 1)]
        if h_pos not in obs and h_pos not in heavy_list:
            heavy_list.append(h_pos)
            i += 1
    j = 0
    while j < num_light:
        l_pos = [random.randint(regions[0][0], regions[0][1] - 1),
                 random.randint(regions[0][2], regions[0][3] - 1)]
        if l_pos not in obs and l_pos not in light_list and l_pos not in heavy_list:
            light_list.append(l_pos)
            j += 1
    return heavy_list, light_list


def initial_specification(obs, agents_h, agents_l, work_regions, states):
    """
    MITL Specifications varphi_i
        - Independent tasks (varphi_i^{own}): ['operator', 'starting time', 'ending time', 'cell']
        - Collaboration tasks (varphi_j^{collab}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'col']
        - Meet tasks (varphi_j^{meet}): ['operator', 'starting time', 'ending time', 'cell', 'Rj', 'meet']
    """
    num = len(agents_h)
    operators = ['G', 'E']
    types = ['', '', '', '', '', '', 'col', 'meet']
    recipients = [i for i in range(num)]
    specs = [[] for i in range(num)]
    for i in range(num):
        num_task = random.randint(2, 3)
        n = 0
        num_meet = 0
        cells = []
        while n < num_task:
            pos = [random.randint(work_regions[i][0], work_regions[i][1] - 1),
                   random.randint(work_regions[i][2], work_regions[i][3] - 1)]
            if pos not in obs and pos not in agents_h and pos not in agents_l:
                cells.append(pos)
                n += 1
        up_t = 5
        for j in range(num_task):
            task = []
            operator = random.choice(operators)
            task.append(operator)
            start = random.randint(up_t, (j + 1) * 40)
            task.append(str(start))
            end = random.randint(start + 1, start + 10)
            task.append(str(end))
            up_t = end
            for key, value in states[i].items():
                if cells[j][0] == value[0] and cells[j][1] == value[1]:
                    task.append(key)
            if num_meet > 0:
                type = random.choice([x for x in types if x != 'meet'])
            else:
                type = random.choice(types)
                if type == 'meet':
                    num_meet = 1
            if type != '':
                recipient = random.choice([x for x in recipients if x != i])
                task.append('R' + str(recipient))
                task.append(type)
            specs[i].append(task)
    return specs


def print_specs(specs):
    spec_print = copy.deepcopy(specs)
    for i in range(len(spec_print)):
        for j in range(len(spec_print[i])):
            if len(spec_print[i][j]) > 4:
                spec_print[i][j][4] = 'R'+str(int(spec_print[i][j][4][1:len(spec_print[i][j][4])])+1)
    return spec_print


def get_tasks(specs, states_work_region):
    tasks = [[] for i in range(3)]
    for i in range(len(specs)):
        for j in range(len(specs[i])):
            if len(specs[i][j]) > 4:
                if specs[i][j][5] == 'col':
                    tasks[1].append(states_work_region[i][specs[i][j][3]])
                if specs[i][j][5] == 'meet':
                    tasks[2].append(states_work_region[i][specs[i][j][3]])
            else:
                tasks[0].append(states_work_region[i][specs[i][j][3]])
    return tasks


def make_world(agents_h, agents_l, r_sens_h, r_sens_l, v_h, v_l, tasks, requests):
    num = len(agents_h)
    agents_h = [np.array(item) for item in agents_h]
    world = World()

    world.agents_heavy = [Agent_heavy() for i in range(num)]
    for i, agent_heavy in enumerate(world.agents_heavy):
        agent_heavy.name = 'H%d' % i
        agent_heavy.p_pos = agents_h[i]
        agent_heavy.r_sens = r_sens_h
        agent_heavy.velocity = v_h
        agent_heavy.path.append(np.array(agents_h[i]))
        agent_heavy.task_left = tasks[i]
        agent_heavy.working_region = i
        agent_heavy.request_send = requests[i]

    world.agents_light = Agent_light()
    world.agents_light.name = 'L%d' % 1
    world.agents_light.p_pos = agents_l
    world.agents_light.r_sens = r_sens_l
    world.agents_light.velocity = v_l
    world.agents_light.path.append(np.array(agents_l))
    world.agents_light.num_round = 0
    world.agents_light.request_list = [[] for i in range(num)]
    return world
