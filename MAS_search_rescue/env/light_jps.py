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

import math
import heapq


def heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def length(current, jump_point):
    return math.sqrt((current[0] - jump_point[0]) ** 2 + (current[1] - jump_point[1]) ** 2)


def total_length(path_list):
    total = 0
    prev = path_list[0]

    for current in path_list:
        if current == prev:
            continue
        total += length(current, prev)

    return total


def whole_path_length(path):
    total = 0
    prev = path[0]
    for current in path:
        if current == prev:
            continue
        total += length(current, prev)
        prev = current
    total = total / 50
    return total


def blocked(c_x, c_y, d_x, d_y, matrix):
    if c_x + d_x < 0 or c_x + d_x >= matrix.shape[0]:
        return True
    if c_y + d_y < 0 or c_y + d_y >= matrix.shape[1]:
        return True
    if d_x != 0 and d_y != 0:
        if matrix[c_x + d_x][c_y] and matrix[c_x][c_y + d_y]:
            return True
        if matrix[c_x + d_x][c_y + d_y]:
            return True
    else:
        if d_x != 0:
            if matrix[c_x + d_x][c_y]:
                return True
        else:
            if matrix[c_x][c_y + d_y]:
                return True
    return False


def dblock(c_x, c_y, d_x, d_y, matrix):
    return matrix[c_x - d_x][c_y] and matrix[c_x][c_y - d_y]


def direction(c_x, c_y, p_x, p_y):
    d_x = int(math.copysign(1, c_x - p_x))
    d_y = int(math.copysign(1, c_y - p_y))
    if c_x - p_x == 0:
        d_x = 0
    if c_y - p_y == 0:
        d_y = 0
    return d_x, d_y


def node_neighbours(c_x, c_y, parent, matrix):
    neighbours = []
    if type(parent) != tuple:
        for i, j in [
            (-1, 0),
            (0, -1),
            (1, 0),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]:
            if not blocked(c_x, c_y, i, j, matrix):
                neighbours.append((c_x + i, c_y + j))

        return neighbours
    d_x, d_y = direction(c_x, c_y, parent[0], parent[1])

    if d_x != 0 and d_y != 0:
        if not blocked(c_x, c_y, 0, d_y, matrix):
            neighbours.append((c_x, c_y + d_y))
        if not blocked(c_x, c_y, d_x, 0, matrix):
            neighbours.append((c_x + d_x, c_y))
        if (
                not blocked(c_x, c_y, 0, d_y, matrix)
                or not blocked(c_x, c_y, d_x, 0, matrix)
        ) and not blocked(c_x, c_y, d_x, d_y, matrix):
            neighbours.append((c_x + d_x, c_y + d_y))
        if blocked(c_x, c_y, -d_x, 0, matrix) and not blocked(
                c_x, c_y, 0, d_y, matrix
        ):
            neighbours.append((c_x - d_x, c_y + d_y))
        if blocked(c_x, c_y, 0, -d_y, matrix) and not blocked(
                c_x, c_y, d_x, 0, matrix
        ):
            neighbours.append((c_x + d_x, c_y - d_y))

    else:
        if d_x == 0:
            if not blocked(c_x, c_y, d_x, 0, matrix):
                if not blocked(c_x, c_y, 0, d_y, matrix):
                    neighbours.append((c_x, c_y + d_y))
                if blocked(c_x, c_y, 1, 0, matrix):
                    neighbours.append((c_x + 1, c_y + d_y))
                if blocked(c_x, c_y, -1, 0, matrix):
                    neighbours.append((c_x - 1, c_y + d_y))

        else:
            if not blocked(c_x, c_y, d_x, 0, matrix):
                if not blocked(c_x, c_y, d_x, 0, matrix):
                    neighbours.append((c_x + d_x, c_y))
                if blocked(c_x, c_y, 0, 1, matrix):
                    neighbours.append((c_x + d_x, c_y + 1))
                if blocked(c_x, c_y, 0, -1, matrix):
                    neighbours.append((c_x + d_x, c_y - 1))
    return neighbours


def jump(c_x, c_y, d_x, d_y, matrix, goal):
    n_x = c_x + d_x
    n_y = c_y + d_y
    if blocked(n_x, n_y, 0, 0, matrix):
        return None

    if (n_x, n_y) == goal:
        return n_x, n_y

    o_x = n_x
    o_y = n_y

    if d_x != 0 and d_y != 0:
        while True:
            if (
                    not blocked(o_x, o_y, -d_x, d_y, matrix)
                    and blocked(o_x, o_y, -d_x, 0, matrix)
                    or not blocked(o_x, o_y, d_x, -d_y, matrix)
                    and blocked(o_x, o_y, 0, -d_y, matrix)
            ):
                return o_x, o_y

            if (
                    jump(o_x, o_y, d_x, 0, matrix, goal) is not None
                    or jump(o_x, o_y, 0, d_y, matrix, goal) is not None
            ):
                return o_x, o_y

            o_x += d_x
            o_y += d_y

            if blocked(o_x, o_y, 0, 0, matrix):
                return None

            if dblock(o_x, o_y, d_x, d_y, matrix):
                return None

            if (o_x, o_y) == goal:
                return o_x, o_y
    else:
        if d_x != 0:
            while True:
                if (
                        not blocked(o_x, n_y, d_x, 1, matrix)
                        and blocked(o_x, n_y, 0, 1, matrix)
                        or not blocked(o_x, n_y, d_x, -1, matrix)
                        and blocked(o_x, n_y, 0, -1, matrix)
                ):
                    return o_x, n_y

                o_x += d_x

                if blocked(o_x, n_y, 0, 0, matrix):
                    return None

                if (o_x, n_y) == goal:
                    return o_x, n_y
        else:
            while True:
                if (
                        not blocked(n_x, o_y, 1, d_y, matrix)
                        and blocked(n_x, o_y, 1, 0, matrix)
                        or not blocked(n_x, o_y, -1, d_y, matrix)
                        and blocked(n_x, o_y, -1, 0, matrix)
                ):
                    return n_x, o_y

                o_y += d_y

                if blocked(n_x, o_y, 0, 0, matrix):
                    return None

                if (n_x, o_y) == goal:
                    return n_x, o_y


def identify_successors(c_x, c_y, came_from, matrix, goal):
    successors = []
    neighbours = node_neighbours(c_x, c_y, came_from.get((c_x, c_y), 0), matrix)

    for cell in neighbours:
        d_x = cell[0] - c_x
        d_y = cell[1] - c_y

        jump_point = jump(c_x, c_y, d_x, d_y, matrix, goal)

        if jump_point is not None:
            successors.append(jump_point)

    return successors


def step_path(result, matrix):
    path = []
    for i in range(len(result) - 1):
        # print("result[i]: ", [result[i][0], result[i][1]], "result[i+1]: ", [result[i+1][0], result[i+1][1]])
        if result[i][0] - result[i + 1][0] == result[i][1] - result[i + 1][1]:
            step = result[i][0] - result[i + 1][0]
            if step > 0:
                # row number decrease, col number decrease - to up left
                # print("S1, step is: ", step)
                for j in range(step):
                    if matrix[result[i][0] - (j+1), result[i][1] - j] == 1:
                        path.append((result[i][0] - j, result[i][1] - j))
                        path.append((result[i][0] - (j+1), result[i][1] - j))
                    else:
                        path.append((result[i][0] - j, result[i][1] - j))
                        path.append((result[i][0] - j, result[i][1] - (j+1)))
            else:
                # print("S2, step is: ", step)
                for j in range(0, step, -1):
                    if matrix[result[i][0] - j, result[i][1] - (j - 1)] == 1:
                        # print("21", (result[i][0] - j, result[i][1] - j), (result[i][0] - (j - 1), result[i][1] - j))
                        path.append((result[i][0] - j, result[i][1] - j))
                        path.append((result[i][0] - (j - 1), result[i][1] - j))
                    else:
                        # print("22", (result[i][0] - j, result[i][1] - j), (result[i][0] - j, result[i][1] - (j - 1)))
                        path.append((result[i][0] - j, result[i][1] - j))
                        path.append((result[i][0] - j, result[i][1] - (j - 1)))

        elif result[i][0] - result[i + 1][0] == result[i + 1][1] - result[i][1]:
            step = result[i][0] - result[i + 1][0]
            if step > 0:
                # row number decreases, col number increase - to up right
                # print("S11, step is: ", step)
                for j in range(step):
                    if matrix[result[i][0] - (j+1), result[i][1] - j] == 1:
                        path.append((result[i][0] - j, result[i][1] + j))
                        path.append((result[i][0] - j, result[i][1] + (j+1)))
                    else:
                        # first to up, then to right
                        path.append((result[i][0] - j, result[i][1] + j))
                        path.append((result[i][0] - (j+1), result[i][1] + j))
            else:
                # row number increases, col number decrease - to bottom left
                # print("S12, step is: ", step)
                for j in range(0, step, -1):
                    if matrix[result[i][0] + j, result[i][1] + (j-1)] == 1:
                        # first left then bottom
                        # print("121", (result[i][0] - j, result[i][1] + j), (result[i][0] - (j - 1), result[i][1] + j))
                        path.append((result[i][0] - j, result[i][1] + j))
                        path.append((result[i][0] - (j-1), result[i][1] + j))
                    else:
                        # first bottom, then left
                        # print("122", (result[i][0] - j, result[i][1] + j), (result[i][0] - j, result[i][1] + (j - 1)))
                        path.append((result[i][0] - j, result[i][1] + j))
                        path.append((result[i][0] - j, result[i][1] + (j - 1)))

        elif result[i][0] - result[i + 1][0] == 0:
            step = result[i][1] - result[i + 1][1]
            if step > 0:
                # print("S3, step is: ", step)
                for j in range(step):
                    path.append((result[i][0], result[i][1] - j))
            else:
                # print("S4, step is: ", step)
                for j in range(0, step, -1):
                    path.append((result[i][0], result[i][1] - j))
        else:
            step = result[i][0] - result[i + 1][0]
            if step > 0:
                # print("S5, step is: ", step)
                for j in range(step):
                    path.append((result[i][0] - j, result[i][1]))
            else:
                # print("S6, step is: ", step)
                for j in range(0, step, -1):
                    # print("6 ", (result[i][0] - j, result[i][1]))
                    path.append((result[i][0] - j, result[i][1]))
    return path


def jump_point_search(matrix, start, goal):
    came_from = {}
    close_set = set()
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}

    pqueue = []
    heapq.heappush(pqueue, (fscore[start], start))

    while pqueue:
        current = heapq.heappop(pqueue)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data

        close_set.add(current)

        successors = identify_successors(
            current[0], current[1], came_from, matrix, goal
        )

        for successor in successors:
            jump_point = successor

            if jump_point in close_set:
                continue
            tentative_g_score = gscore[current] + length(current, jump_point)

            if tentative_g_score < gscore.get(jump_point, 0) or jump_point not in [j[1] for j in pqueue]:
                came_from[jump_point] = current
                gscore[jump_point] = tentative_g_score
                fscore[jump_point] = tentative_g_score + heuristic(jump_point, goal)
                heapq.heappush(pqueue, (fscore[jump_point], jump_point))
    return None
