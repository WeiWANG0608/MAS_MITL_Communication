"""
Author Wei Wang (wei7@kth.se)

This file is used to generate weighted transition systems upon the initialized environment for instance solving.
"""

import numpy as np
import math

W_I = 1
W_SE = 1
W_NW = 1


class TransitionSystem:
    """
    # already satisfy never W_obs in states(): self.map[pi, pj] != 1
    """
    def __init__(self, map, doc_declaration, wts_decl, init_location):
        self.map = map
        self.doc_decl = doc_declaration
        self.ta_decl = wts_decl
        self.row, self.col = self.map.shape
        self.init_loc = init_location
        self.states = self.states_WTS()
        self.locations = self.generate_locations_WTS()
        self.labels_location = self.locations_invariant_WTS()
        self.edges = self.generate_edges_WTS()

    def states_WTS(self):
        states = dict()
        s = 0
        # Player position
        for pi in range(self.map.shape[0]):
            for pj in range(self.map.shape[1]):
                # already satisfy never W_obs
                if self.map[pi, pj] != 1:
                    states['w'+str(s)] = [pi, pj]
                    s += 1
        return states

    def get_state(self, val):
        for key, value in self.states.items():
            if val == value:
                return key
        return "key doesn't exist"

    def generate_locations_WTS(self):
        """
        location_i = [location_i (name), rate of exponential, x_i_inUppaal, y_i_inUppaal, initial (true/false)]
        """
        locations = []
        for i in self.states:
            if self.states[i][0] == self.init_loc[0] and self.states[i][1] == self.init_loc[1]:
                locations.append([i, "null", str(self.states[i][0]), str(self.states[i][1]), "true"])
                print("The initial position is ", i)
            else:
                locations.append([i, "null", str(self.states[i][0]), str(self.states[i][1]), "false"])

        return locations

    def locations_invariant_WTS(self):
        labels_loc = []
        for i in range(len(self.states)):
            labels_loc.append(["null"])
        return labels_loc

    def generate_edges_WTS(self):
        """
        edge_ab = [[location_a(name), location_b(name)], [_Guard], [_Sync], Update(weight), [x_la, y_la], [x_lb, y_lb]]
        """
        addDeclaration(self.doc_decl, ['int', 'w'])
        edges = list()
        for ci in range(self.row):
            for cj in range(self.col):
                for ni in range(ci, self.row):
                    for nj in range(cj, self.col):
                        # act = Idle
                        if self.map[ci, cj] != 1 and self.map[ni, nj] != 1 and cj == nj and ci == ni:
                            edges.append([self.get_state([ci, cj]), self.get_state([ni, nj]), "null", "null",
                                          "w=w+" + str(W_I)])
                        # act = S or E
                        if self.map[ci, cj] != 1 and self.map[ni, nj] != 1 and (
                                (ni - ci == 1 and nj == cj) or (nj - cj == 1 and ni == ci)):
                            edges.append([self.get_state([ci, cj]), self.get_state([ni, nj]), "null", "null",
                                          "w=w+"+str(W_SE)])
        for ci in range(self.row - 1, -1, -1):
            for cj in range(self.col - 1, -1, -1):
                for ni in range(self.row - 1, -1, -1):
                    for nj in range(self.col - 1, -1, -1):
                        # act = N or W
                        if self.map[ci, cj] != 1 and self.map[ni, nj] != 1 and (
                                (ci - ni == 1 and cj == nj) or (cj - nj == 1 and ci == ni)):
                            edges.append([self.get_state([ci, cj]), self.get_state([ni, nj]), "null", "null",
                                          "w=w+"+str(W_NW)])

        return edges


def addDeclaration(decl, val):
    if val[0] in decl:
        if len(decl[val[0]]) > 0 and (val[1] not in decl[val[0]][0: len(decl[val[0]])-1].split(',')):
            decl[val[0]] = decl[val[0]][0: len(decl[val[0]])-1]
            decl[val[0]] = decl[val[0]]+','+val[1]+';'
    else:
        decl[val[0]] = val[1]+';'

    return decl


def total_states(map):
    states = dict()
    s = 0
    # Player position
    for pi in range(map.shape[0]):
        for pj in range(map.shape[1]):
            # already satisfy never W_obs
            if map[pi, pj] != 1:
                states['w'+str(s)] = [pi, pj]
                s += 1
    return states
