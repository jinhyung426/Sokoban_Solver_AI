#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    total_distance = 0;

    for box in state.boxes:
        local_distance = 0;
        if box not in state.storage:
            local_distance = state.width + state.height + 1;
            for storage in state.storage:
                compare_distance = abs(storage[0]-box[0]) + abs(storage[1]-box[1])
                if compare_distance < local_distance:
                    local_distance = compare_distance
        total_distance += local_distance

    return total_distance


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def is_at_corner(state, box):

    if (box[0] == 0 and box[1] == 0) or (box[0] == 0 and box[1] == state.height - 1) or (box[0] == state.width - 1 or box[1] == 0) or (box[0] == state.width - 1 and box[1] == state.height - 1):
        return True

    adj_obstacles = []
    for obstacle in state.obstacles:
        b_o_distance = abs(box[0] - obstacle[0]) + abs(box[1] - obstacle[1])
        if b_o_distance == 1:
            adj_obstacles.append(obstacle)

    if len(adj_obstacles) >= 3:
        return True

    if len(adj_obstacles) == 2:
        if (adj_obstacles[0][0] != adj_obstacles[1][0]) and (
                adj_obstacles[0][1] != adj_obstacles[1][1]):
            return True

    return False

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    total_distance = 0;

    for box in state.boxes:
        box_goal_distance = 0;
        robot_box_distance = 0;
        used_storage = {}
        used_robot = {}
        duplicate = 0

        if box not in state.storage:

            if is_at_corner(state, box):
                total_distance += 2 * state.width + 2 * state.height + len(state.storage)
                continue

            box_goal_distance = state.width + state.height + 1;
            robot_box_distance = state.width + state.height + 1;

            for storage in state.storage:
                compare_distance = abs(storage[0] - box[0]) + abs(storage[1] - box[1])

                if compare_distance < box_goal_distance:
                    box_goal_distance = compare_distance
                    if storage in used_storage.values():
                        duplicate += 2
                        used_storage[box] = storage
                    else:
                        used_storage[box] = storage


            for robot in state.robots:
                compare_distance2 = abs(robot[0] - box[0]) + abs(robot[1] - box[1])

                if compare_distance2 < robot_box_distance:
                    robot_box_distance = compare_distance2
                    if robot in used_robot.values():
                        duplicate += 2
                        used_robot[box] = robot
                    else:
                        used_robot[box] = robot

        total_distance += box_goal_distance
        total_distance += robot_box_distance
        total_distance += duplicate

    return total_distance


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.

    fval = sN.hval * weight + sN.gval
    return fval


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    search_start_time = os.times()[0]
    real_timebound = search_start_time + timebound

    se = SearchEngine('custom', 'default')
    wrapped_function = (lambda sN : fval_function(sN, weight + 0.4))
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_function)

    goal_state = False
    res = se.search(timebound)

    if res is False:
        return res
    is_first_loop = True
    i = 0.4;

    while search_start_time < real_timebound:

        if res is False:
            return goal_state

        time_spent = os.times()[0] - search_start_time
        search_start_time = os.times()[0]

        timebound -= time_spent

        if is_first_loop or res.gval <= costbound[0]:
            is_first_loop = False
            costbound = [res.gval, res.gval, res.gval]
            goal_state = res

        wrapped_function = (lambda sN: fval_function(sN, weight - i))
        i += 0.4
        se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_function)

        res = se.search(timebound, costbound)

    return goal_state



def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    search_start_time = os.times()[0]
    real_timebound = search_start_time + timebound

    se = SearchEngine('best_first', 'default')
    se.init_search(initial_state, sokoban_goal_state, heur_fn)

    goal_state = False
    res = se.search(timebound)

    if res is False:
        return res
    is_first_loop = True

    while search_start_time < real_timebound:

        if res is False:
            return goal_state

        time_spent = os.times()[0] - search_start_time
        search_start_time = os.times()[0]

        timebound -= time_spent

        if is_first_loop or res.gval <= costbound[0]:
            is_first_loop = False
            costbound = [res.gval, res.gval, res.gval]
            goal_state = res
        res = se.search(timebound, costbound)

    return goal_state
