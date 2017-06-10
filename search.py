# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())

  stack = util.Stack()
  node = Node(problem.getStartState(), None, None, 0)
  stack.push(node)
  explored = []
  while (not stack.isEmpty()):
    node = stack.pop() 
    state = node.getState()
    explored.append(state)
    if (problem.isGoalState(state)):
      return node.getActionsToNode()
      print ('Done')
    else:
      successors = problem.getSuccessors(state)
      for item in successors:
        state = item[0]
        action = item[1]
        const = item[2]
        newnode = Node(state,node,action,const)
        if state not in explored:
          stack.push(newnode)
      
  """procedure DFS-iterative(G,v):
  2      let S be a stack
  3      S.push(v)
  4      while S is not empty
  5          v = S.pop()
  6          if v is not labeled as discovered:
  7              label v as discovered
  8              for all edges from v to w in G.adjacentEdges(v) do 
  9                  S.push(w)"""

  util.raiseNotDefined()

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  queue = util.Queue()
  node = Node(problem.getStartState(), None, None, 0)
  queue.push(node)
  explored = []
  while (not queue.isEmpty()):
    node = queue.pop() 
    state = node.getState()
    explored.append(state)
    if (problem.isGoalState(state)):
      return node.getActionsToNode()
    else:
      successors = problem.getSuccessors(state)
      for item in successors:
        state = item[0]
        action = item[1]
        const = item[2]
        newnode = Node(state,node,action,const)
        if state not in explored:
          queue.push(newnode)

  util.raiseNotDefined()
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  queue = util.PriorityQueue()  
  node = Node(problem.getStartState(), None, None, 0)
  queue.push(node, problem.getCostOfActions(node.getActionsToNode()) + heuristic(node.getState(),problem))
  explored = []
  while (not queue.isEmpty()):
    node = queue.pop() 
    state = node.getState()
    explored.append(state)
    if (problem.isGoalState(state)):
      return node.getActionsToNode()
    else:
      successors = problem.getSuccessors(state)
      for item in successors:
        state = item[0]
        action = item[1]
        const = item[2]
        newnode = Node(state,node,action,const)
        if state not in explored:
          queue.push(newnode, problem.getCostOfActions(newnode.getActionsToNode()) + heuristic(newnode.getState(),problem))


  """// The distance from start to a neighbor
  tentative_gScore := gScore[current] + dist_between(current, neighbor)
  if tentative_gScore >= gScore[neighbor]
      continue		// This is not a better path.

  // This path is the best until now. Record it!
  cameFrom[neighbor] := current
  gScore[neighbor] := tentative_gScore
  fScore[neighbor] := gScore[neighbor] + heuristic_cost_estimate(neighbor, goal)
  """

  util.raiseNotDefined()

class Node:
    def __init__(self, state, parent, action, stepcost):
        self.state  = state
        self.parent = parent
        self.action = action
        if parent==None:
            self.cost = stepcost
        else:
            self.cost = parent.cost + stepcost
        if parent==None:
            self.actionsToNode = []
        else:
            t = parent.actionsToNode[:]
            t.append(action)
            self.actionsToNode = t

    def __str__(self):
        return "State: " + str(self.state) + "\n" + \
               "Parent: " + str(self.parent.state) + "\n" + \
               "Action: " + str(self.action) + "\n" + \
               "Cost: " + str(self.cost)

    def getState(self):
        return self.state

    def getParent(self):
        return self.parent

    def getAction(self):
        return self.action

    def getCost(self):
        return self.cost

    def pathFromStart(self):
        stateList = []
        actionList = []
        currNode = self
        while currNode.getAction() is not None:
            #print stateList
            #print actionList
            stateList.append(currNode.getState())
            actionList.append(currNode.getAction())
            currNode = currNode.parent
        actionList.reverse()
        return actionList

    def getActionsToNode(self):
        return self.actionsToNode
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
