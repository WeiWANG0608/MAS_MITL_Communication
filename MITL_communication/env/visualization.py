"""
Author Wei Wang (wei7@kth.se)

This file is used to plot and visualize the results
- draw_map(): draw the map of environment with obstacles and all agents
- draw_path(): plot the trajectories of all agents
- make_video_mp4(): make a video of the execution procedure
- animate_solution(): generate animations of the execution procedure
"""

import copy
import matplotlib.pyplot as plt
import numpy as np
from IPython import display
import math
import os
import moviepy.video.io.ImageSequenceClip
import matplotlib
import seaborn as sns

matplotlib.rcParams['savefig.dpi'] = 200

# Some colours
WHITE = '#FFFFFF'
BLACK = '#000000'
GREEN = '#009E73'
LIGHT_GREEN = '#90EE90'
BLUE = '#0072B2'
LIGHT_BLUE = '#56B4E9'
PINK = '#CC79A7'
ORANGE = '#D55E00'
LIGHT_ORANGE = '#E69F00'
GREY = '#999999'
RED = '#EE4000'
YELLOW = '#F0E442'

col_map = {0: WHITE, 1: BLACK, 2: GREEN, 3: BLUE, 4: LIGHT_BLUE, 5: PINK,
           6: ORANGE, 7: LIGHT_ORANGE, 8: GREY, 9: RED, 10: LIGHT_GREEN, -1: YELLOW}


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    UNDERLINE = '\033[4m'


def draw_map(map, tasks, num_heavy, folder):
    rows, cols = map.shape
    for i in range(len(tasks[0])):
        map[tasks[0][i][0]][tasks[0][i][1]] = 8
    for i in range(len(tasks[1])):
        map[tasks[1][i][0]][tasks[1][i][1]] = 6
    for i in range(len(tasks[2])):
        map[tasks[2][i][0]][tasks[2][i][1]] = 5

    # Create figure of the size of the maze
    plt.figure(1, figsize=(cols * 0.5, rows * 0.5))

    # Remove the axis ticks and add title title
    ax = plt.gca()
    ax.set_title('The Map of 2 * ' + str(int(num_heavy / 2)) + ' work regions ( ' + str(rows) + ' * ' + str(cols) +
                 ' Each WR ' + str(int(rows / 2)) + '*' + str(int(cols / (num_heavy / 2))) + ' )', fontsize=20)
    ax.set_xticks([])
    ax.set_yticks([])

    # Give a color to each cell
    colored_maze = [[col_map[map[j, i]] for i in range(cols)] for j in range(rows)]

    # Create figure of the size of the maze
    plt.figure(1, figsize=(cols, rows))

    # Create a table to color
    grid = plt.table(cellText=None,
                     cellColours=colored_maze,
                     cellLoc='center',
                     loc=(0, 0),
                     edges='closed')
    # Modify the hight and width of the cells in the table
    tc = grid.properties()['children']
    for cell in tc:
        cell.set_height(1.0 / rows)
        cell.set_width(1.0 / cols)
    plt.savefig(folder + "/" + "Map.png", dpi=100)
    plt.show()


def draw_path(map_in, path, folder, title):
    maze = copy.deepcopy(map_in)
    rows, cols = maze.shape

    if 'Round' in title:
        maze = np.zeros((rows, cols), dtype=int)

    plt.figure(figsize=(10, 10), dpi=100)

    # Remove the axis ticks and add title title
    ax = plt.gca()
    ax.set_title(title, fontsize=20)
    ax.set_xticks([])
    ax.set_yticks([])

    # Give a color to each cell
    colored_maze = [[col_map[maze[j, i]] for i in range(cols)] for j in range(rows)]

    # Create a table to color
    grid = plt.table(cellText=None,
                     cellColours=colored_maze,
                     cellLoc='center',
                     loc=(0, 0),
                     edges='closed')
    # Modify the hight and width of the cells in the table
    if 'robot' in title:
        bcmap = sns.mpl_palette("Blues", len(path))
        for idx, b in enumerate(bcmap):
            grid[path[idx][0], path[idx][1]].set_facecolor(b)
    else:
        for i in range(len(path)):
            grid[path[i][0], path[i][1]].set_facecolor(GREY)

    tc = grid.properties()['children']
    for cell in tc:
        cell.set_height(1.0 / rows)
        cell.set_width(1.0 / cols)
    plt.savefig(folder + "/" + title + ".png", dpi=100)
    plt.show()
    plt.clf()


def make_video_mp4(filepath):
    image_folder = filepath
    video_name = filepath + '/' + 'video.mp4'
    fps = 1
    image_files = [os.path.join(image_folder, img)
                   for img in sorted(os.listdir(image_folder))
                   if img.endswith(".png")]
    clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(image_files, fps=fps)
    clip.write_videofile(video_name)


def animate_solution(map_ori, path, region, task, r_sens_l, r_sens_h, v_l, folder, video=False):
    maze = copy.deepcopy(map_ori)
    newfolder = folder + '/' + "video"
    if not (os.path.exists(newfolder)):
        os.makedirs(newfolder)
    # Size of the maze
    rows, cols = maze.shape

    # creating grid for subplots
    fig = plt.figure()

    # Give a color to each cell
    colored_maze = [[col_map[maze[j, i]] for i in range(cols)] for j in range(rows)]

    # Create a table to color
    grid = plt.table(cellText=None,
                     cellColours=colored_maze,
                     cellLoc='center',
                     loc=(0, 0),
                     edges='closed')
    grid.set_fontsize(30)
    # Modify the hight and width of the cells in the table
    tc = grid.properties()['children']
    for cell in tc:
        cell.set_height(1.0 / rows)
        cell.set_width(1.0 / cols)

    # Update the color at each frame
    for i in range(len(path)):

        ax1 = plt.subplot2grid(shape=(3, 5), loc=(0, 0), rowspan=3, colspan=3)

        colored_maze = [[col_map[maze[j, i]] for i in range(cols)] for j in range(rows)]

        # Create a table to color
        grid = ax1.table(cellText=None,
                         cellColours=colored_maze,
                         cellLoc='center',
                         loc=(0, 0),
                         edges='closed')
        grid.set_fontsize(40)
        # Modify the hight and width of the cells in the table
        tc = grid.properties()['children']
        for cell in tc:
            cell.set_height(1.0 / rows)
            cell.set_width(1.0 / cols)

        # Remove the axis ticks and add title title
        # ax1 = plt.gca()
        if i > 0:
            ax1.set_title('Simulation at step ' + str(math.floor(i / v_l)), fontsize=20)
        else:
            ax1.set_title('Simulation at step ' + str(0), fontsize=20)
        ax1.set_xticks([])
        ax1.set_yticks([])

        a_sens_l = [p for p in range(-r_sens_l, r_sens_l + 1)]
        a_sens_h = [p for p in range(-r_sens_h, r_sens_h + 1)]
        a_light = []
        a_heavy = [[] for i in range(int((len(path[0]) - 2) / 2))]
        a_heavy_pre = [[] for i in range(int((len(path[0]) - 2) / 2))]
        overlap = []

        for l in a_sens_l:
            for j in a_sens_l:
                if 0 <= path[i][0] + l < rows and 0 <= path[i][1] + j < cols and \
                        math.sqrt(abs(l) ** 2 + abs(j) ** 2) <= r_sens_l and \
                        maze[path[i][0] + l, path[i][1] + j] != 1 and maze[path[i][0] + l, path[i][1] + j] != 5 and \
                        maze[path[i][0] + l, path[i][1] + j] != 6 and maze[path[i][0] + l, path[i][1] + j] != 8:
                    a_light.append([path[i][0] + l, path[i][1] + j])

        for l in a_sens_h:
            for j in a_sens_h:
                for k in range(2, len(path[i]), 2):
                    if 0 <= path[i][k] + l < rows and 0 <= path[i][k + 1] + j < cols and \
                            math.sqrt(abs(l) ** 2 + abs(j) ** 2) <= r_sens_h and \
                            maze[path[i][k] + l, path[i][k + 1] + j] != 1 and \
                            maze[path[i][k] + l, path[i][k + 1] + j] != 5 and \
                            maze[path[i][k] + l, path[i][k + 1] + j] != 6 and \
                            maze[path[i][k] + l, path[i][k + 1] + j] != 8:
                        a_heavy[int((k - 2) / 2)].append([path[i][k] + l, path[i][k + 1] + j])
                    if i > 0 and 0 <= path[i - 1][k] + l < rows and 0 <= path[i - 1][k + 1] + j < cols:
                        a_heavy[int((k - 2) / 2)].append([path[i - 1][k] + l, path[i - 1][k + 1] + j])

        for j in range(len(a_heavy)):
            if len(a_heavy_pre[j]) > 0:
                for k in a_heavy_pre[j]:
                    if maze[k[0], k[1]] != 1 and maze[k[0], k[1]] != 5 and \
                            maze[k[0], k[1]] != 6 and maze[k[0], k[1]] != 8:
                        grid.get_celld()[(k[0], k[1])].set_facecolor(WHITE)
            for k in a_heavy[j]:
                if maze[k[0], k[1]] != 1 and maze[k[0], k[1]] != 5 and \
                        maze[k[0], k[1]] != 6 and maze[k[0], k[1]] != 8:
                    grid.get_celld()[(k[0], k[1])].set_facecolor(LIGHT_GREEN)
            for k in a_light:
                if maze[k[0], k[1]] != 1 and maze[k[0], k[1]] != 5 and \
                        maze[k[0], k[1]] != 6 and maze[k[0], k[1]] != 8:
                    grid.get_celld()[(k[0], k[1])].set_facecolor(LIGHT_BLUE)

            for element in a_heavy[j]:
                if element in a_light:
                    overlap.append(element)
                    print(f"f{bcolors.WARNING}Overlap H[", j, "] and Light", overlap)

        if i > 0:
            if maze[path[i - 1][0], path[i - 1][1]] == 1:
                grid.get_celld()[(path[i - 1][0], path[i - 1][1])].set_facecolor(BLACK)
            for k in range(2, len(path[i]), 2):
                if maze[path[i - 1][k], path[i - 1][k + 1]] in [5, 6, 8]:
                    maze[path[i - 1][k], path[i - 1][k + 1]] = 0
                    grid.get_celld()[(path[i - 1][k], path[i - 1][k + 1])].set_facecolor(WHITE)

        for k in overlap:
            grid.get_celld()[(k[0], k[1])].set_facecolor(YELLOW)
        grid.get_celld()[(path[i][0], path[i][1])].set_facecolor(BLUE)
        if i > 0:
            grid.get_celld()[(path[i][0], path[i][1])].get_text().set_text(
                'U ' + str(math.floor(i / v_l)) + "."
                + str(i % v_l))
        else:
            grid.get_celld()[(path[i][0], path[i][1])].get_text().set_text('U ' + str(0))

        for k in range(2, len(path[i]), 2):
            grid.get_celld()[(path[i][k], path[i][k + 1])].set_facecolor(GREEN)
            grid.get_celld()[(path[i][k], path[i][k + 1])].get_text().set_text(
                'R' + str(int((k - 2) / 2) + 1) + ' ' + str(math.floor(i / v_l)))

        ax2 = plt.subplot2grid(shape=(3, 5), loc=(0, 3), rowspan=3, colspan=2)
        ax2.axis('tight')
        ax2.axis('off')
        collabel = ("Robot", "Task Region", "Task left")
        d0 = [["R" + str(j + 1)] for j in range(len(task))]
        d1 = []
        for j in range(len(task)):
            d1.append([int(item) + 1 for item in region[j][i].split(',')])
        # d1 = [[region[j][i]] for j in range(len(task))]
        d2 = [task[j][i] for j in range(len(task))]
        data = []
        for k in range(len(d0)):
            data.append([d0[k], d1[k], d2[k]])
        info = ax2.table(cellText=data, colLabels=collabel, loc='center')
        info.set_fontsize(30)

        ax2.set_title('Information', fontsize=20)

        # automatically adjust padding horizontally
        # as well as vertically.
        plt.tight_layout()

        plt.savefig(newfolder + "/" + '{:0>4}'.format(i) + ".png", dpi=100)
        display.display(fig)
        display.clear_output(wait=True)
        # display plot
        # plt.show()

    if video:
        make_video_mp4(newfolder)
