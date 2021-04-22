import sys
import puzz
import pdqpq

MAX_SEARCH_ITERS = 100000
GOAL_STATE = puzz.EightPuzzleBoard("012345678")

def is_not_empty(list_of_states):
    return len(list_of_states)

def bfs(start_state, results):
    frontier = pdqpq.PriorityQueue()
    explored = set()
    parent = {}
    count = 0
    frontier.add(start_state, count)
    count +=1
    parent[start_state] = ("start", None)

    if start_state == GOAL_STATE:
        results['path'].insert(0, (parent[start_state][0], start_state))
        results['frontier_count'] = count
        results['expanded_count'] = len(explored)
        return results

    while is_not_empty(frontier):
        curr_state = frontier.pop()
        #error checking to be done for pop
        explored.add(curr_state)
        succ_state = curr_state.successors()
        print(succ_state)
        for move, child_state in succ_state.items():
            if child_state not in frontier and child_state not in explored:
                parent[child_state] = (move, curr_state)
                if child_state == GOAL_STATE:
                    while child_state is not None:
                        results['path'].insert(0, (parent[child_state][0], child_state))
                        results['path_cost'] += edge_cost(parent[child_state][1], child_state)
                        child_state = parent[child_state][1]
                    results['frontier_count'] = count
                    results['expanded_count'] = len(explored)
                    return results
                else:
                    frontier.add(child_state, count)
                    count +=1
    no_solution = {}
    no_solution['frontier_count'] = results['frontier_count']
    no_solution['expanded_count'] = len(explored)
    return no_solution   

def best_first_search(start_state, results, heuristic, search_type):
    frontier = pdqpq.PriorityQueue()
    explored = set()
    parent = {}
    frontier.add(start_state, 0)
    results['frontier_count'] += 1
    parent[start_state] = ("start", None)
    while is_not_empty(frontier):
        curr_state_cost = frontier.pq[0][0]
        curr_state = frontier.pop()
        if search_type == 'astar' and curr_state != start_state:
            curr_state_cost -= heuristic(curr_state)
        if curr_state == GOAL_STATE:
            while curr_state is not None:
                    results['path'].insert(0, (parent[curr_state][0], curr_state))
                    results['path_cost'] += edge_cost(parent[curr_state][1], curr_state)
                    curr_state = parent[curr_state][1]
            results['expanded_count'] = len(explored)
            return results 
        explored.add(curr_state)
        succ_state = curr_state.successors()
        for move, child_state in succ_state.items():
            child_cost = 0
            if search_type == 'ucost':
                child_cost = curr_state_cost + edge_cost(curr_state, child_state)
            elif search_type == 'greedy':
                child_cost = heuristic(child_state)
            elif search_type == 'astar':
                child_cost = curr_state_cost + edge_cost(curr_state, child_state) + heuristic(child_state)
            if child_state not in frontier and child_state not in explored:
                parent[child_state] = (move, curr_state)
                frontier.add(child_state, child_cost)
                results['frontier_count'] += 1
            elif child_state in frontier and frontier.get(child_state) > child_cost:
                parent[child_state] = (move, curr_state)
                frontier.add(child_state, child_cost)
    no_solution = {}
    no_solution['frontier_count'] = results['frontier_count']
    no_solution['expanded_count'] = len(explored)
    return no_solution 

def edge_cost(parent_state, child_state):
    if parent_state is None:
       return 0
    x, y = parent_state.find(puzz.BLANK_CHAR)
    tile_value = child_state._get_tile(x, y)
    cost = int(tile_value) ** 2
    return cost

def h1(curr_state):
    cost = 0
    for x in range(0, 3):
        for y in range(0, 3):
            tile_value = curr_state._get_tile(x,y)
            if tile_value != puzz.BLANK_CHAR and tile_value != GOAL_STATE._get_tile(x,y):
                cost += 1
    return cost

def h2(curr_state):
    cost = 0
    for x in range(0, 3):
        for y in range(0, 3):
            tile_value = curr_state._get_tile(x,y)
            if tile_value != puzz.BLANK_CHAR:
                x1, y1 = GOAL_STATE.find(tile_value)
                cost += abs(x - x1) + abs(y - y1)
    return cost

def h3(curr_state):
    cost = 0
    for x in range(0, 3):
        for y in range(0, 3):
            tile_value = curr_state._get_tile(x,y)
            if tile_value != puzz.BLANK_CHAR:
                x1, y1 = GOAL_STATE.find(tile_value)
                cost += (int(tile_value) ** 2) * (abs(x - x1) + abs(y - y1))
    return cost

def solve_puzzle(start_state, stratergy):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state: an EightPuzzleBoard object indicating the start state for the search
        flavor: a string indicating which type of search to run.  Can be one of the following:
            'bfs' - breadth-first search
            'ucost' - uniform-cost search
            'greedy-h1' - Greedy best-first search using a misplaced tile count heuristic
            'greedy-h2' - Greedy best-first search using a Manhattan distance heuristic
            'greedy-h3' - Greedy best-first search using a weighted Manhattan distance heuristic
            'astar-h1' - A* search using a misplaced tile count heuristic
            'astar-h2' - A* search using a Manhattan distance heuristic
            'astar-h3' - A* search using a weighted Manhattan distance heuristic
    
    Returns: 
        A dictionary containing describing the search performed, containing the following entries:
            'path' - a list of EightPuzzleBoard objects indicating the path from the start state 
                to the goal state (both should be included).  Omitted if the search fails.
            'path_cost' - the total cost of the path, taking into account the costs associated 
                with each state transition.  Omitted if the search fails.
            'frontier_count' - the number of unique states added to the search frontier at any
                point during the search.
            'expanded_count' - the number of unique states removed from the frontier and expanded 
                (successors generated).
    """

    results = {
        'path': [],
        'path_cost': 0,
        'frontier_count': 0,
        'expanded_count': 0,
    }
    # 
    # fill in the function body here
    #
    stratergy = stratergy.lower()
    search_type = stratergy.split('-')
    if search_type[0] == 'bfs':
        results = bfs(start_state, results)
    elif search_type[0] == 'ucost':
        results = best_first_search(start_state, results, None, search_type[0])
    else:
        if search_type[1] == 'h1':
            results = best_first_search(start_state, results, h1, search_type[0])
        elif search_type[1] == 'h2':
            results = best_first_search(start_state, results, h2, search_type[0])
        elif search_type[1] == 'h3':
            results = best_first_search(start_state, results, h3, search_type[0])
    return results

def print_summary(results):
    if 'path' in results:
        print("found solution of length {}, cost {}".format(len(results['path']), 
                                                            results['path_cost']))
        for move, state in results['path']:
            print("  {:5} {}".format(move, state))
    else:
        print("no solution found")
    print("{} states placed on frontier, {} states expanded".format(results['frontier_count'], 
                                                                    results['expanded_count']))


############################################

if __name__ == '__main__':

    start = puzz.EightPuzzleBoard(sys.argv[1])
    method = sys.argv[2]

    print("solving puzzle {} -> {}".format(start, GOAL_STATE))
    results = solve_puzzle(start, method)
    print_summary(results)
