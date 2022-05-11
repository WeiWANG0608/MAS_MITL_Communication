"""
Author Wei Wang (wei7@kth.se)

class heavy_path: used to translate the MITL specifications to a file that UPPAAL can recognize
                  and generate the network of timed automata, as well as translate the WTS into
                  a file for instance solving.

- get_work_map(): get the map of each work regions
- get_path_cell2coord(): get the path from cell name to coordinate
- if_collab_meet_promise(): check the request is collab or meet, if meet request then whether it needs promise
- get_promise_request(): get the meet promise and meet request
- get_cooperation_map(): get the new map after accepting the requests
- get_update_path(): get the new path upon the specification after accepting the requests
- get_independent_requests_tasks(): get the independent task and independent requests
"""

import time
from datetime import datetime
import numpy as np
from heavy_WTS import TransitionSystem, total_states
from heavy_MITL2TBA import MITL2TBA
import subprocess
import json
import copy
import os

dirname = os.path.dirname
ABPATH = os.path.join(dirname(dirname(__file__)))


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


class heavy_path:

    """
    Generates file "flag.txt" to identify the current verification agent Rx
    Generates file "system_infoRx.txt" for independent task verification of Rx by UPPAAL
    Generates file "system_infoCRx.txt" for independent task + independent requests verification of Rx by UPPAAL
    Read "system_dataRx.json", which is the WTS Ti generated from workspace and TBA Ai generated from specification

    /output/...
    Generate file "pathoutputRx.txt" if there is a path for Rx under independent task
    Generate file "pathoutputCRx.txt" if there is a path for Rx under independent task + independent requests
    Generate file "(repo)Rx.xml/.xtr" by UPPAAL if there is a path for Rx under independent task
    Generate file "(repo)CRx.xml/.xtr" by UPPAAL if there is a path for Rx under independent task + independent requests
    """

    def __init__(self, map_ori, specifications, filefolder, flag, init_pos=0):
        self.map = map_ori
        self.spec_list = specifications  # [['G', '15', '18', 'c13'], ['E', '32', '40', 'c16', 'R1']]
        self.filefolder = filefolder
        self.flag = flag
        self.init_pos = init_pos
        self.states_wts, self.state_wt = self.get_product()
        # self.path = self.get_path()

    def declaration_toString(self, decl_dict):
        decl = ''
        for i in decl_dict:
            decl = decl + i + ' ' + decl_dict[i] + '\n'
        if decl == '':
            decl = 'null'
        return decl

    def get_product(self):
        doc_declaration = dict()
        wts_declaration = dict()
        ta_declaration = dict()

        # # heavy-duty robot module
        wts_heavy = TransitionSystem(self.map, doc_declaration, wts_declaration, self.init_pos)
        state_wts = wts_heavy.states_WTS()
        locations_wts = wts_heavy.generate_locations_WTS()
        labels_wts = wts_heavy.locations_invariant_WTS()
        edges_wts = wts_heavy.generate_edges_WTS()

        ta_heavy = MITL2TBA(state_wts, locations_wts, labels_wts, edges_wts, self.spec_list, doc_declaration,
                            ta_declaration, 0)
        state_ta = ta_heavy.states_TA()
        locations_ta = ta_heavy.generate_locations_TA()
        labels_ta = ta_heavy.locations_invariant_TA()
        edges_ta = ta_heavy.get_product_wts()


        text_file = open(self.filefolder + "/" + 'flag' + ".txt", "w")
        text_file.write(self.flag)
        text_file.close()

        "#  &"
        doc_system = "ProcessWTS() = WTS();\nProcessTA() = TA();\nsystem ProcessWTS, ProcessTA;"
        doc_decl = self.declaration_toString(doc_declaration)

        temp_decl = self.declaration_toString(wts_declaration) + '#' + self.declaration_toString(ta_declaration)
        temp_list = 'WTS#TA'

        info_all = doc_system + '&' + doc_decl + '&' + temp_decl + '&' + temp_list
        text_file = open(self.filefolder + "/" + 'system_info' + self.flag + ".txt", "w")
        text_file.write(info_all)
        text_file.close()

        # for visualization
        for i in range(len(locations_wts)):
            locations_wts[i][2] = str(int(locations_wts[i][2]) * 200)
            locations_wts[i][3] = str(int(locations_wts[i][3]) * 200)

        locations = {'wts': locations_wts, 'ta': locations_ta}
        labels = {'wts': labels_wts, 'ta': labels_ta}
        edges = {'wts': edges_wts, 'ta': edges_ta}

        data = {"locations": locations,
                "labels": labels,
                "edges": edges}

        with open(self.filefolder + "/" + 'system_data' + self.flag +'.json', 'w') as outfile:
            json.dump(data, outfile, indent=4)
        outfile.close()
        subprocess.run([ABPATH + '/deploy.sh'])

        return state_wts, state_ta

    def get_path(self):
        with open(self.filefolder + '/output/' + 'pathoutput' + self.flag + '.txt') as f:
            lines = f.readlines()
        path = []
        if len(lines) > 0:
            path = [item.strip() for item in lines]
        # print("heavy_path_generation ", path)
        return path


def get_work_map(map_ori, map_list, work_region):
    map_row, map_col = map_ori.shape
    new_map = np.ones((map_row, map_col), dtype=int)
    for pi in range(map_list[work_region][0], map_list[work_region][1]):
        for pj in range(map_list[work_region][2], map_list[work_region][3]):
            new_map[pi, pj] = map_ori[pi, pj]
    return new_map


def get_path_cell2coord(path, states_dict):
    path_coord = []
    for i in range(len(path)):
        path_coord.append(states_dict[path[i]])
    return path_coord


def if_collab_meet_promise(spec):
    ifcollab = False
    ifmeet = False
    ifpromise = False
    if len(spec) > 4:
        if spec[5] == 'col':
            ifcollab = True
        else:
            ifmeet = True
            if spec[0] == "E":
                ifpromise = True
    return ifcollab, ifmeet, ifpromise


def get_promise_request(spec, index, map_local, filefolder, flag, init_p):
    veri_spec = copy.deepcopy(spec)
    if veri_spec[index][1] < veri_spec[index][2]:
        for i in range(1, int(veri_spec[index][2]) - int(veri_spec[index][1])):
            print("******** Start getting the promise ********")
            env_heavy = heavy_path(map_local, veri_spec, filefolder, flag, init_p)
            now = datetime.now().time()  # time object
            print("Python time ", now)
            time.sleep(1)
            print("1s later")
            now = datetime.now().time()  # time object
            print("Python time after sleep() ", now)
            path_heavy_promise = env_heavy.get_path()
            # print("The file length is :", len(path_heavy_promise))
            if len(path_heavy_promise) > 0:
                print("======== Find the best promise ========")
                promise = veri_spec[index]
                request = copy.deepcopy(veri_spec[index])
                request[0] = 'E'
                print("Meet promise is ", promise, ". Meet request is ", request)
                break
            else:
                print("======== old ========")
                veri_spec[index][1] = str(int(veri_spec[index][1]) + 1)
    return promise, request


def get_cooperation_map(map_ori, map_list, coop_region, target_h):
    print("The cooperative task is in region ", coop_region, " the needed robot is ", target_h)
    reg_r = int(len(map_ori)/2)
    new_map = copy.deepcopy(map_ori)
    region_list = [i for i in range(len(map_list))]
    for i in coop_region:
        region_list.remove(i)
    region_list.remove(target_h)
    print("The current map contains the follow task regions ", coop_region, target_h)
    for i in region_list:
        for pi in range(map_list[i][0], map_list[i][1]):
            for pj in range(map_list[i][2], map_list[i][3]):
                # the boundary of work regions are accessible
                if pi != reg_r-1 and pi != reg_r:
                    new_map[pi, pj] = 1
    # draw_map_simple(new_map, 'open boundary')
    return new_map


def get_update_path(current_pos, current_time, coop_region, request, recipient, task_left,
                  state_dict_local, map_ori, map_list, filefolder):
    """
    current_pos is the coordination in map_ori
    get the cells from the old specification to new one
    request: [['G', '67', '70', 'w15', '1', 'col'], [...],...]
    """
    coop_path = []
    map_coop = get_cooperation_map(map_ori, map_list, coop_region, recipient)
    state_dict_new = total_states(map_coop)

    # get the current coordinate in old map
    for key, value in state_dict_local[recipient].items():
        if current_pos[0] == value[0] and current_pos[1] == value[1]:
            curent_pos_old_map = key

    # get the current coordinate in new map
    for key, value in state_dict_new.items():
        if current_pos[0] == value[0] and current_pos[1] == value[1]:
            curent_pos_new_map = key

    # get the new positions of all old task haven't done yet in the new map
    for i in range(len(task_left)):
        loc_pre = state_dict_local[recipient][task_left[i][3]]
        for key, value in state_dict_new.items():
            if loc_pre == value:
                task_left[i][3] = key

    # get the positions of requests in the new map
    for i in range(len(request)):
        loc_pre = state_dict_local[int(request[i][4])][request[i][3]]
        for key, value in state_dict_new.items():
            if loc_pre == value:
                request[i][3] = key

        task_left += request

    spec_new = copy.deepcopy(task_left)

    flag = "CR" + str(recipient)
    print(f"{bcolors.WARNING}The new specification without time updating is ", spec_new)
    print(f"{bcolors.FAIL}The current position before updating is ", curent_pos_old_map, "after updating is ", curent_pos_new_map)
    for i in range(len(spec_new)):
        spec_new[i][1] = str(max(int(spec_new[i][1]) - current_time, 0))
        spec_new[i][2] = str(max(int(spec_new[i][2]) - current_time, 0))
    if spec_new[0][2] == '0':
        spec_new.pop(0)
    print(f"{bcolors.ENDC}The new specification after the time update is ", spec_new)
    path_heavy = heavy_path(map_coop, spec_new, filefolder, flag, current_pos).get_path()

    if len(path_heavy) > 0:
        print(f"{bcolors.OKBLUE}The cooperation can be achieved! ")
        coop_path = get_path_cell2coord(path_heavy, state_dict_new)

    else:
        print(f"{bcolors.FAIL}Tried, but cannot achieve cooperation")

    return coop_path, spec_new, state_dict_new


def get_independent_requests_tasks(specs, map_regions, agents_h, file_folder):
    num = len(specs)
    independent_requests = [[[] for j in range(num)] for i in range(num)]
    independent_tasks = [[] for i in range(num)]
    for i in range(num):
        flag = "R" + str(i)
        for k in range(len(specs[i])):
            ifcollab, ifmeet, ifpromise = if_collab_meet_promise(specs[i][k])
            if not ifcollab and not ifmeet:
                independent_tasks[i].append(specs[i][k])
            # collab requests
            elif ifcollab:
                j = int(specs[i][k][4][1:len(specs[i][k][4])])
                independent_requests[i][j].append(specs[i][k])
            # meet requests
            elif ifmeet and not ifpromise:
                independent_tasks[i].append(specs[i][k][0:4])
                j = int(specs[i][k][4][1:len(specs[i][k][4])])
                independent_requests[i][j].append(specs[i][k])
            elif ifmeet and ifpromise:
                veri_spec = copy.deepcopy(specs[i])
                veri_spec[k][0] = 'G'
                print("The spec to verify (before) is ", veri_spec)
                meet_promise_candi = veri_spec[k]
                veri_spec = [item for item in veri_spec if (len(item) == 4 or (len(item) > 4 and item[5] != 'col'))]
                if len(independent_tasks[i]) > 0:
                    veri_spec[0:k] = independent_tasks[i]
                veri_path = heavy_path(map_regions[i], veri_spec, file_folder, flag, agents_h[i]).get_path()
                if len(veri_path) > 0:
                    j = int(specs[i][k][4][1:len(specs[i][k][4])])
                    independent_requests[i][j].append(specs[i][k])
                    independent_tasks[i].append(meet_promise_candi)
                else:
                    index = veri_spec.index(meet_promise_candi)
                    meet_promise, meet_request = get_promise_request(veri_spec, index, map_regions[i], file_folder,
                                                                     flag, agents_h[i])
                    j = int(meet_request[4][1:len(meet_request[4])])
                    independent_requests[i][j].append(meet_request)
                    independent_tasks[i].append(meet_promise)

    return independent_tasks, independent_requests
