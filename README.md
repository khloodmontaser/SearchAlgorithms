# SearchAlgorithms
implement four search algorithms (BFS,DFS,UCS,A*)
#######search class
# search.py

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):

    # Initialize the fringe with the start state and an empty list of steps.
    
    fringe = util.Stack()  # Create a stack data structure to keep track of the nodes to be explored.
    expanded = set()  # Create a set to keep track of the states that have already been expanded.
    fringe.push((problem.getStartState(), []))  # Add the start state and an empty list of steps to the fringe.

    while not fringe.isEmpty():
        # Pop the topmost node from the fringe.
        state, steps = current_node = fringe.pop()

        # If the state has already been expanded, skip this node.
        if state in expanded:
            continue

        # Mark the state as expanded and check if it is a goal state.
        expanded.add(state)
        if problem.isGoalState(state):
            return steps

        # Generate all successors of the current node and add them to the fringe.
        for successor, action, stepCost in problem.getSuccessors(state):
            if successor not in expanded:
                # Add the successor and the action that led to it to the fringe.
                fringe.push((successor, steps + [action]))

    # If no goal state is found, return None.
    return None

def breadthFirstSearch(problem):
    # Initialize the fringe with the start state and an empty list of actions.

    fringe = util.Queue()  # Create a queue data structure to keep track of the nodes to be explored.
    visited = set()  # Create a set to keep track of the states that have already been visited.
    fringe.push((problem.getStartState(), []))  # Add the start state and an empty list of actions to the fringe.

    while not fringe.isEmpty():
        # Pop the leftmost node from the fringe.
        state, actions = fringe.pop()

        # If the state has already been visited, skip this node.
        if state in visited:
            continue

        # Mark the state as visited and check if it is a goal state.
        visited.add(state)
        if problem.isGoalState(state):
            return actions

        # Generate all successors of the current node and add them to the fringe.
        for successor in problem.getSuccessors(state):
            next_state, action, _ = successor
            if next_state not in visited:
                # Add the next state and the list of actions to the fringe.
                next_actions = actions + [action]
                fringe.push((next_state, next_actions))

    # If no goal state is found, return None.
    return None

def uniformCostSearch(problem):

    # Initialize the fringe with the start state and an empty list of actions, with a priority of 0.
    fringe = util.PriorityQueue()
    visited = set()
    fringe.push((problem.getStartState(), [], 0), 0)

    while not fringe.isEmpty():
        # Pop the node with the lowest path cost from the fringe.
        state, actions, cost = fringe.pop()

        # If the state has already been visited, skip this node.
        if state in visited:
            continue

        # Mark the state as visited and check if it is a goal state.
        visited.add(state)
        if problem.isGoalState(state):
            return actions

        # Generate all successors of the current node and add them to the fringe with their path cost as the priority.
        for successor in problem.getSuccessors(state):
            next_state, action, step_cost = successor
            if next_state not in visited:
                next_actions = actions + [action]
                next_cost = cost + step_cost
                fringe.push((next_state, next_actions, next_cost), next_cost)

    # If no goal state is found, return None.
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  
    # Import the Directions module from the game package
    from game import Directions

    # Initialize a priority queue to store the search frontier
    fringe = util.PriorityQueue()

    # Initialize an empty list to keep track of the visited states
    visitedList = []

    # Push the starting point into the queue with priority num of 0
    # The starting point is represented by problem.getStartState() method,
    # an empty list [] to keep track of the path from the starting point,
    # and a cost of 0.
    fringe.push((problem.getStartState(),[],0),0 + heuristic(problem.getStartState(),problem))

    # Pop out the next state with the lowest priority from the queue,
    # assign the state, path, and cost of the path to three variables,
    # and add the state and its total cost to the visited list.
    (state,toDirection,toCost) = fringe.pop()
    visitedList.append((state,toCost + heuristic(problem.getStartState(),problem)))

    # Continue the loop until the goal state is found.
    while not problem.isGoalState(state):

        # Get the successors of the current state using the problem.getSuccessors(state) method.
        successors = problem.getSuccessors(state)

        # Iterate over each successor
        for son in successors:

            # Initialize a flag to indicate if the successor has been visited before
            visitedExist = False

            # Calculate the total cost of the path to the successor
            total_cost = toCost + son[2]

            # Iterate over the visited list to check if the successor has been visited before
            # and has a lower cost than the previous visit.
            for (visitedState,visitedToCost) in visitedList:
                if (son[0] == visitedState) and (total_cost >= visitedToCost): 
                    visitedExist = True
                    break

            # If the successor has not been visited before or has a lower cost than the previous visit,
            # push it into the queue with a priority equal to the sum of the actual cost from the starting point to the successor,
            # the cost of the path from the starting point to the successor,
            # and the estimated cost from the successor to the goal point.
            # Add the successor and its total cost to the visited list.
            if not visitedExist:        
                fringe.push((son[0],toDirection + [son[1]],toCost + son[2]),toCost + son[2] + heuristic(son[0],problem)) 
                visitedList.append((son[0],toCost + son[2]))

        # Pop out the next state with the lowest priority from the queue,
        # assign the state, path, and cost of the path to three variables.
        (state,toDirection,toCost) = fringe.pop()

    # Return the path to the goal state.
    return toDirection

#Abbreviations 
bfs = breadthFirstSearch

dfs = depthFirstSearch

astar = aStarSearch

ucs = uniformCostSearch
