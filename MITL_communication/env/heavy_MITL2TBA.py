"""
Author Wei Wang (wei7@kth.se)

This file is used to translate the MITL specifications to a file that UPPAAL can recognize.
"""

import random


class specification(object):
    def __init__(self):
        self.operator = ''
        self.start_time = 0
        self.end_time = 0
        self.task = ''
        self.ifCoop = False
        self.coop = ''
        self.clock = ''
        self.sync = []             # the channel of the order
        self.update = ''           # the bool value of \phi
        self.detected = False


class MITL2TBA:

    def __init__(self, states_wts, locations_wts, labels_wts, edges_wts, inputSpec, doc_declaration, ta_decl, init):

        self.ori_spec = inputSpec  # [['G', '15', '18', 'c13'], ['E', '32', '40', 'c16', 'R1']]
        self.doc_decl = doc_declaration
        self.ta_decl = ta_decl
        self.init_loc_ta = init
        self.spec_trans = self.split_specification()
        self.states_ta = self.states_TA()
        self.states_wts = states_wts
        self.locations_ta = self.generate_locations_TA()
        self.locations_wts = locations_wts
        self.labels_ta = self.locations_invariant_TA()
        self.labels_wts = labels_wts
        self.edges_ta = self.generate_edges_TA()
        self.edges_wts = edges_wts

    def eventually(self, spec):
        """
        eventually: find all the transitions in edges_wts to the task cell, add Sync: p_i!
                             the transition in edges_ta to satisfy state, add Sync: p_i?
                                                                              Update: veri_i:= (time interval)? true:false

        edge_ab = [[location_a(name), location_b(name)], [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]]
        """
        for i in range(len(self.edges_wts)):
            # print("eventually", spec.task, self.edges_wts[i][0][0], self.edges_wts[i][0][1])
            if spec.task == self.edges_wts[i][1] and self.edges_wts[i][0] != self.edges_wts[i][1]:
                self.edges_wts[i][3] = spec.sync[0] + '!'
                # print("eve", self.edges_wts[i][3])
            if spec.task == self.edges_wts[i][0] and self.edges_wts[i][0] != self.edges_wts[i][1]:
                self.edges_wts[i][3] = spec.sync[1] + '!'
                # print("globally", self.edges_wts[i][3])

        for i in range(len(self.edges_ta)):
            # print("eventually", spec.clock[3], self.edges_ta[i][0], self.edges_ta[i][1])
            if int(spec.clock[2]) == int(self.edges_ta[i][1][1:len(self.edges_ta[i][1])]) and \
                    self.edges_ta[i][0] != self.edges_ta[i][1]:
                self.edges_ta[i][3] = spec.sync[0] + '?'
                self.edges_ta[i][4] = spec.update + ":=(w>=" + spec.start_time + "&& w<=" + spec.end_time + \
                                      ")? true: false"
                # print("eve", self.edges_ta[i][3], self.edges_ta[i][4])
            if int(spec.clock[2:len(spec.clock)]) == int(self.edges_ta[i][0][1:len(self.edges_ta[i][1])]) and \
                    self.edges_ta[i][0] != self.edges_ta[i][1]:
                self.edges_ta[i][3] = spec.sync[1] + '?'
                # print("globally", self.edges_wts[i][3])

        return None

    def globally(self, spec):
        """
        globally: find all the transitions in edges_wts to the task cell, add Sync: p_i!
                           the transitions in edges_wts leave the task cell, add Sync: np_i!
                           the loop transition in edges_wts at task cell, add Sync: p_i!
                           the transition in edges_ta to satisfy state, add Sync: p_i?
                                                                          update: cl_i:=0
                           the transition in edges_ta leave satisfy state, add Sync: np_i?
                           the loop transition in edges_ta at satisfy state, add Sync: p_i?
                                                                                 Update: cl_i = cl_i+1
                                                                                 veri_i:= (cl_i == end_t-start_t && time interval)? true:false
                                                                                 [10, 15]: t>= 15
        """
        for i in range(len(self.edges_wts)):
            # print("task is ", spec.task, " source is ", self.edges_wts[i][0], " target is ", self.edges_wts[i][1])
            if spec.task == self.edges_wts[i][1]:
                self.edges_wts[i][3] = spec.sync[0] + '!'
                # print("globally", self.edges_wts[i][3])
            if spec.task == self.edges_wts[i][0] and self.edges_wts[i][0] != self.edges_wts[i][1]:
                self.edges_wts[i][3] = spec.sync[1] + '!'
                # print("globally", self.edges_wts[i][3])

        for i in range(len(self.edges_ta)):
            if int(spec.clock[2:len(spec.clock)]) == int(self.edges_ta[i][1][1:len(self.edges_ta[i][1])]):
                if self.edges_ta[i][0] != self.edges_ta[i][1]:
                    self.edges_ta[i][2] = "w>=" + str(int(spec.start_time)-1)
                    self.edges_ta[i][3] = spec.sync[0] + '?'
                    self.edges_ta[i][4] = spec.clock + ":=0"
                else:
                    self.edges_ta[i][3] = spec.sync[0] + '?'
                    self.edges_ta[i][4] = spec.clock + "=" + spec.clock + "+1~" + spec.update + ":=(" + spec.clock + \
                                          "==" + str(int(spec.end_time) - int(spec.start_time)) + " && w>=" + \
                                          spec.start_time + " && w<=" + spec.end_time + ")? true: false"
                # print("globally", self.edges_ta[i][3], self.edges_ta[i][4])
            if int(spec.clock[2:len(spec.clock)]) == int(self.edges_ta[i][0][1:len(self.edges_ta[i][1])]) and \
                    self.edges_ta[i][0] != self.edges_ta[i][1]:
                self.edges_ta[i][3] = spec.sync[1] + '?'
                # print("globally", self.edges_wts[i][3])

        return None

    def split_specification(self):
        """Input:    self.ori_spec = [['G', '15', '18', 'c13'], ['E', '32', '40', 'c16', 'R1']]
           Output    Object [] - obj.clock: cl_i (clock name, int) start from 1
                               - obj.operator: E, G
                               - obj.start_time: int
                               - obj.end_time: int
                               - obj.task: cell (location) name
                               - obj.sync: p_i (if the ap is sent: pi! /receive: pi?)
                               - obj.update: veri_i
                               if cooperative:
                                 - obj.ifCoop: True (otherwise False)
                                 - obj.coop: the robot need to cooperate
        """
        speclist = [specification() for i in range(len(self.ori_spec))]
        for i, spec in enumerate(speclist):
            spec.clock = 'cl' + str(i + 1)
            addDeclaration(self.ta_decl, ['int', 'cl' + str(i + 1)])
            spec.operator = self.ori_spec[i][0]
            spec.start_time = self.ori_spec[i][1]
            spec.end_time = self.ori_spec[i][2]
            spec.task = self.ori_spec[i][3]
            spec.sync = ['p' + str(i + 1), 'np' + str(i + 1)]
            addDeclaration(self.doc_decl, ['chan', 'p' + str(i + 1)])
            addDeclaration(self.doc_decl, ['chan', 'np' + str(i + 1)])
            spec.update = 'veri' + str(i + 1)
            addDeclaration(self.doc_decl, ['bool', 'veri' + str(i + 1)])
            if len(self.ori_spec[i]) > 4:
                spec.ifCoop = True
                spec.coop = self.ori_spec[i][4]
        return speclist

    def states_TA(self):
        states = dict()
        for i in range(len(self.spec_trans) + 1):
            states['l' + str(i)] = [i * random.randint(5, 20) * 10, i * random.randint(5, 20) * 10]
        # print(states)
        return states

    def get_state_TA(self, val):
        for key, value in self.states_ta.items():
            if val == value:
                return key
        return "key doesn't exist"

    def generate_locations_TA(self):
        """
        location_i = [location_i (name), rate of exponential, x_i_inUppaal, y_i_inUppaal, initial (true/false)]
        """
        locations = []
        for i in self.states_ta:
            if int(i[1:len(i)]) == self.init_loc_ta:
                locations.append([i, "null", str(self.states_ta[i][0]), str(self.states_ta[i][1]), "true"])
            else:
                locations.append([i, "null", str(self.states_ta[i][0]), str(self.states_ta[i][1]), "false"])

        return locations

    def locations_invariant_TA(self):
        labels_loc = []
        for i in range(len(self.states_ta)):
            labels_loc.append(["null"])
        return labels_loc

    def generate_edges_TA(self):
        """
        edge_ab = [[location_a(name), location_b(name)], [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]]
        """
        edges = list()
        for i in self.states_ta:
            if i != 'l0':
                edges.append(['l0', i, "null", "null", "null"])
                edges.append([i, 'l0', "null", "null", "null"])
                edges.append([i, i, "null", "null", "null"])

        return edges

    def get_product_wts(self):
        for i, spec in enumerate(self.spec_trans):
            if self.spec_trans[i].operator == 'G':
                self.globally(self.spec_trans[i])
            if self.spec_trans[i].operator == 'E':
                self.eventually(self.spec_trans[i])

        return self.edges_ta

    # def addCoopRequest(edges):
    #     return edges


def addDeclaration(decl, input):
    if input[0] in decl:
        if len(decl[input[0]]) > 0 and (input[1] not in decl[input[0]][0: len(decl[input[0]]) - 1].split(',')):
            decl[input[0]] = decl[input[0]][0: len(decl[input[0]]) - 1]
            decl[input[0]] = decl[input[0]] + ',' + input[1] + ';'
    else:
        decl[input[0]] = input[1] + ';'

    return decl
