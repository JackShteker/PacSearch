import sys

import util
import Queue
import heapq

class SearchProblem:

    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def getSuccessors(self, state):
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        util.raiseNotDefined()

def depthFirstSearch(problem):
    visited = set()
    max_depth = 0
    def rec(state, seq, depth):
        if problem.isGoalState(state):
            return seq[:], depth
        for next_state, action, cost in problem.getSuccessors(state):
            if next_state not in visited:
                visited.add(next_state)
                seq.append(action)
                ans, max_depth = rec(next_state, seq, depth + 1)
                seq.pop()
                if ans is not None:
                    return ans, max_depth
        return None, None
    seq, max_depth = rec(problem.getStartState(), [], 1)
    if seq is None:
        print("There is no path")

    memory = sys.getsizeof(visited)
    print("Memory for visited: {0}, max recursion depth: {1}".format(memory, max_depth))

    print(seq)
    return seq

def breadthFirstSearch(problem):
    predecessor = {problem.getStartState():(None, None)}
    q = Queue.Queue()
    q.put(problem.getStartState())
    final = None
    max_len = 0
    bal = 1
    while not q.empty():
        max_len = max(max_len, bal)
        state = q.get()
        bal -= 1
        for next_state, action, cost in problem.getSuccessors(state):
            if next_state not in predecessor:
                predecessor[next_state] = (state, action)
                q.put(next_state)
                bal += 1
                if problem.isGoalState(next_state):
                    final = next_state
        if final is not None:
            break

    path = []
    while True:
        if predecessor[final][0] is None:
            break
        path.append(predecessor[final][1])
        final = predecessor[final][0]
    path.reverse()

    memory = sys.getsizeof(predecessor)
    print("Memory for predecessor: {0}, max queue length: {1}".format(memory, max_len))

    print(path)
    return path

def nullHeuristic(state, problem=None):
    return 0

def manhattanHeuristic(position, problem):
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem):
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

def aStarSearch(problem, heuristic=nullHeuristic):

    predecessor = {problem.getStartState():(None, None)}
    gscores = {}
    gscores[problem.getStartState()] = 0
    fscores = {}
    fscores[problem.getStartState()] = heuristic(problem.getStartState(), problem)
    heap = [(fscores[problem.getStartState()], problem.getStartState())]

    while heap:
        fscore, state = heapq.heappop(heap)

        if problem.isGoalState(state):
            path = []
            while True:
                if predecessor[state][0] is None:
                    break
                path.append(predecessor[state][1])
                state = predecessor[state][0]
            path.reverse()
            return path

        for next_state, action, cost in problem.getSuccessors(state):
            score = gscores[state] + cost
            if next_state not in gscores or score < gscores[next_state]:
                predecessor[next_state] = state, action
                gscores[next_state] = score
                fscores[next_state] = score + heuristic(next_state, problem)
                heapq.heappush(heap, (fscores[next_state], next_state))
    return None

def greedySearch(problem, heuristic=nullHeuristic):

    predecessor = {problem.getStartState():(None, None)}
    heap = [(heuristic(problem.getStartState(), problem), problem.getStartState())]

    while heap:
        score, state = heapq.heappop(heap)

        if problem.isGoalState(state):
            path = []
            while True:
                if predecessor[state][0] is None:
                    break
                path.append(predecessor[state][1])
                state = predecessor[state][0]
            path.reverse()
            return path

        for next_state, action, cost in problem.getSuccessors(state):
            if next_state not in predecessor:
                predecessor[next_state] = state, action
                heapq.heappush(heap, (heuristic(next_state, problem), next_state))
    return None






bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
greedy = greedySearch

