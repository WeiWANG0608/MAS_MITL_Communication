import random


class AgentEnv:
    # Actions
    STAY = 0
    MOVE_LEFT = 1
    MOVE_RIGHT = 2
    MOVE_UP = 3
    MOVE_DOWN = 4

    # Give names to actions
    actions_names = {
        STAY: "stay",
        MOVE_LEFT: "move left",
        MOVE_RIGHT: "move right",
        MOVE_UP: "move up",
        MOVE_DOWN: "move down"
    }

    def __init__(self, map_shape, p_h):
        """ Constructor of the environment Maze.
        """
        self.map_row_s, self.map_row_e, self.map_col_s, self.map_col_e = map_shape
        self.actions = self.__actions()
        self.pos_heavy = p_h

    def __actions(self):
        actions = dict()
        actions[self.STAY] = (0, 0)
        actions[self.MOVE_LEFT] = (0, -1)
        actions[self.MOVE_RIGHT] = (0, 1)
        actions[self.MOVE_UP] = (-1, 0)
        actions[self.MOVE_DOWN] = (1, 0)
        return actions

    def move(self):
        """ Player makes a step in the maze, given a current position and an action.
            If the action STAY or an inadmissible action is used, the player stays in place.
            Simultaneously the minotaur makes the move

            for_transition_prob --
                returns the len(l) of valid minotaur positions to set t_prob to 1/l
            :return tuple next_state:
                (Px,Py,Mx,My) on the maze that player and minotaur transitions to.
        """
        # For the player
        # Compute the future position given current (state, action)
        action = random.choice(list(self.actions.keys()))

        row = self.pos_heavy[0] + self.actions[action][0]
        col = self.pos_heavy[1] + self.actions[action][1]

        hitting_maze_walls = (row == self.map_row_s - 1) or (row == self.map_row_e) or \
                             (col == self.map_col_s - 1) or (col == self.map_col_e)
        # or (self.maze[row, col] == 1)
        # Based on the impossiblity check return the next state.

        if hitting_maze_walls:
            return [self.pos_heavy[0], self.pos_heavy[1]]
        else:
            return [row, col]
