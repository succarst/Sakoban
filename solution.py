import math
import os # for time functions
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

    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    distance_sum = 0
    for box in state.boxes:
        minimum_distance = math.inf #set minimum distance to infinity
        for storage in state.storage:
            manhattan_dist = (abs(box[0] - storage[0]) + abs (box[1] - storage[1])) #calculate manhattan distance between storage and box
            if manhattan_dist < minimum_distance:
                minimum_distance = manhattan_dist
        distance_sum += minimum_distance
    return distance_sum


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

def heur_alternate(state):
# IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.


    heuristic = 0

    # First we want to figure out where the obstacles are
    obstacles = []

    for x in range(-1, state.width):
        obstacles.append((x, -1))
        obstacles.append((x, state.height))
    for y in range(-1, state.height):
        obstacles.append((-1, y))
        obstacles.append((state.width, y))
    for z in state.obstacles:
        obstacles.append(z)

    # Then we iterate through each box
    for box in state.boxes:
        if (box[0] + 1, box[1]) in obstacles and (box[0], box[1] + 1) in obstacles and box not in state.storage:
            heuristic = math.inf
        if (box[0] - 1, box[1]) in obstacles and (box[0], box[1] + 1) in obstacles and box not in state.storage:
            heuristic = math.inf
        if (box[0] + 1, box[1]) in obstacles and (box[0], box[1] - 1) in obstacles and box not in state.storage:
            heuristic = math.inf
        if (box[0] - 1, box[1]) in obstacles and (box[0], box[1] - 1) in obstacles and box not in state.storage:
            heuristic = math.inf

        # next we want to iterate through the storage locations and robot locations and store their distances
        distance = []

        for storage in state.storage:
            distance.append(abs(storage[0] - box[0]) + abs(storage[1] - box[1]))
        for robots in state.robots:
            distance.append(abs(robots[0] - box[0]) + abs(robots[1] - box[1]))

        heuristic += min(distance) # Increment by smallest distance

    for robots in state.robots:
        robot_shortest_distance = math.inf
        for boxes in state.boxes:
            robot_box_distance = abs(robots[0] - boxes[0]) + abs(robots[1] - boxes[1])
            robot_shortest_distance = min(robot_shortest_distance, robot_box_distance)
        heuristic += robot_shortest_distance

    return heuristic

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
# IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    # Many searches will explore nodes (or states) that are ordered by their f-value.
    # For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    # You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    # The function must return a numeric f-value.
    # The value will determine your state's position on the Frontier list during a 'custom' search.
    # You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
    # IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    # initialize the search engine
    search_engine = SearchEngine('custom', 'full')
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))

    # initialize time
    current_time = os.times()[0]
    end_time = current_time + timebound
    time_left = timebound

    # initialize bounds and variables
    costbound = (math.inf, math.inf, math.inf)
    best_path = False
    result = search_engine.search(timebound)

    while current_time < end_time:
        time_left -= (os.times()[0] - current_time)
        current_time = os.times()[0]
        if result is False:
            return best_path
        if result.gval <= costbound[0]:
            costbound = (result.gval, math.inf, result.gval)
            best_path = result
        result = search_engine.search(time_left, costbound)
    return best_path

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    # Initialize the search engine
    search_engine = SearchEngine('best_first', 'full')
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn)

    # initialize time
    current_time = os.times()[0]
    time_left = timebound

    # initialize costbound and result
    final_result = False
    costbound = (math.inf, math.inf, math.inf) # Initially set costbound to infinity

    while time_left > 0:
        current_result = search_engine.search(time_left, costbound)
        if current_result:
            time_left -= (os.times()[0] - current_time)
            current_time = os.times()[0] # get current time
            costbound = (current_result.gval, math.inf, math.inf)
            final_result = current_result
        else:
            return final_result
    return final_result

