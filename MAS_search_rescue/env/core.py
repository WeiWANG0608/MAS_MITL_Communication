"""
Author Wei Wang (wei7@kth.se)

This file is used to define the features and parameters of all agents
"""

class Entity(object):
    def __init__(self):
        self.name = ''
        self.p_pos = None
        self.size = 1
        self.r_sens = 0
        self.velocity = 0
        self.path = []


class Agent_light(Entity):

    def __init__(self):
        super(Agent_light, self).__init__()
        self.region = None
        self.num_round = 0
        self.request_list = []


class Agent_heavy(Entity):
    """
    task.name; p_pos;
    task.completed
    """

    def __init__(self):
        super(Agent_heavy, self).__init__()
        self.region2go = []
        self.request_send = []
        self.request_receive = []
        self.task_left = []
        self.working_region = ''
        self.collab = []
        self.detected = False


class World(object):

    def __init__(self):
        self.agents_light = Agent_light()
        self.agents_heavy = []


