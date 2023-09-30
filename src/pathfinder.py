'''
CMSI 2130 - Homework 1
Author: Grayson von Goetz und Schwanenfliess

Modify only this file as part of your submission, as it will contain all of the logic
necessary for implementing the A* pathfinder that solves the target practice problem.
'''
import queue
from maze_problem import MazeProblem
from dataclasses import *
from typing import *

@dataclass
class SearchTreeNode:
    """
    SearchTreeNodes contain the following attributes to be used in generation of
    the Search tree:

    Attributes:
        player_loc (tuple[int, int]):
            The player's location in this node.
        action (str):
            The action taken to reach this node from its parent (or empty if the root).
        parent (Optional[SearchTreeNode]):
            The parent node from which this node was generated (or None if the root).
    """
    player_loc: tuple[int, int]
    action: str
    parent: Optional["SearchTreeNode"]
    targets_left: set[tuple[int, int]]
    cost: int

    def __lt__(self, other: "SearchTreeNode") -> bool:
        return self.cost < other.cost
    
    def __hash__(self) -> int:
        return hash((self.player_loc, self.action, frozenset(self.targets_left)))
    
    def __eq__(self, other: "object") -> bool:
        if not isinstance(other, SearchTreeNode):
            return NotImplemented
        return self.player_loc == other.player_loc and self.action == other.action and self.targets_left == other.targets_left


def pathfind(problem: "MazeProblem") -> Optional[list[str]]:
    """
    The main workhorse method of the package that performs A* graph search to find the optimal
    sequence of actions that takes the agent from its initial state and shoots all targets in
    the given MazeProblem's maze, or determines that the problem is unsolvable.

    Parameters:
        problem (MazeProblem):
            The MazeProblem object constructed on the maze that is to be solved or determined
            unsolvable by this method.

    Returns:
        Optional[list[str]]:
            A solution to the problem: a sequence of actions leading from the
            initial state to the goal (a maze with all targets destroyed). If no such solution is
            possible, returns None.
    """

    visited: set["SearchTreeNode"] = set()
    frontier: queue.PriorityQueue[Tuple[int, "SearchTreeNode"]] = queue.PriorityQueue()
    frontier.put((0, SearchTreeNode(problem.get_initial_loc(),
                                    "", 
                                    None, 
                                    problem.get_initial_targets(), 
                                    0)))

    while not frontier.empty():
        node: "SearchTreeNode" = frontier.get()[1]
        
        if len(node.targets_left) == 0:
            return going_back(node)
        
        if node in visited:
            continue
        
        transitions = problem.get_transitions(node.player_loc, 
                                              node.targets_left)
        for action in transitions:
            moves: Any = transitions.get(action)
            targets_hit: set[tuple[int, int]] = moves.get("targets_hit")

            new_remaining_targets = node.targets_left.copy()
            for target in targets_hit:
                if target in new_remaining_targets:
                    new_remaining_targets.remove(target)

            child_node: "SearchTreeNode" = SearchTreeNode(moves.get("next_loc"), 
                                                          action, 
                                                          node, 
                                                          new_remaining_targets.copy(),
                                                          node.cost + moves.get("cost"))

            frontier.put((node.cost + len(node.targets_left) + heuristic(node), child_node))

        visited.add(node)

    return None

def going_back(end_node: "SearchTreeNode") -> list[str]:
    """
    A helper method that returns the solution path from the end node to the start node.
    
    Parameters: 
        end_node (SearchTreeNode):
            The end node of the pathfinding algorithm.
    
    Returns:
        list[str]:
            The solution path from the end node to the start node.
    """

    solution = []

    while end_node.parent is not None:
        solution.append(end_node.action)
        end_node = end_node.parent

    solution.reverse()

    return solution

def heuristic(node: "SearchTreeNode") -> int:
    """
    A heuristic function that estimates the minimum cost to reach the goal state
    based on the Manhattan distance to the nearest remaining target.

    Parameters:
        node (SearchTreeNode):
            The current search tree node for which the heuristic is computed.

    Returns:
        int:
            The estimated cost (heuristic value) to reach the goal state.
    """
    if len(node.targets_left) == 0:
        return 0

    player_loc = node.player_loc
    nearest_target = min(node.targets_left, 
                         key=lambda target: abs(target[0] - player_loc[0]) + abs(target[1] - player_loc[1]))

    return abs(nearest_target[0] - player_loc[0]) + abs(nearest_target[1] - player_loc[1])