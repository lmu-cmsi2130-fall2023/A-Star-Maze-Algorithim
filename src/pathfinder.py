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
    player_loc: Tuple[int, int]
    action: str
    parent: Optional["SearchTreeNode"]
    remaining_targets: Set[Tuple[int, int]]
    cost: int
    
    def __lt__(self, other:object) -> Any:
        return self.cost < other.cost

    def __hash__(self) -> int:
        return hash((self.player_loc, self.action, frozenset(self.remaining_targets)))
     # TODO: Add any other attributes and method overrides as necessary!
    
    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, SearchTreeNode):
            return NotImplemented
        return self.player_loc == __value.player_loc and self.action == __value.action and self.remaining_targets == __value.remaining_targets and self.targets_shot == __value.targets_shot

def solution(last_node: SearchTreeNode) -> list[str]:
    solution= []
    
    while last_node.parent is not None:
        solution.append(last_node.action)
        last_node = last_node.parent

    solution.reverse()

    return solution

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
    
    frontier: queue.PriorityQueue [Tuple[int, "SearchTreeNode"]]= queue.PriorityQueue()
    visited: set["SearchTreeNode"] = set()
    frontier.put((0, SearchTreeNode(problem.get_initial_loc(), "", None, problem.get_initial_targets(), 0)))
    
    while not frontier.empty():
        node = frontier.get()[1]
        if node not in visited:
            visited.add(node.player_loc)
            if len(node.remaining_targets) == 0:
                return solution(node)
            transitions = problem.get_transitions(node.player_loc, node.remaining_targets)
            
            
            for action in transitions:
                shoot_options = transitions.get(action)
                targets_shot: Set[Tuple[int, int]] = shoot_options.get("targets_shot")
                new_targets_left = node.remaining_targets.copy()
                
                for remainder in new_targets_left:
                    if remainder in new_targets_left:
                        targets_shot.remove(remainder)
                        
                child_node: SearchTreeNode = SearchTreeNode(shoot_options.get("player_loc"), action, node, targets_shot, targets_shot, node.cost + shoot_options.get("cost"))
                frontier.put((child_node.cost, child_node))
                
            visited.add(node)

                
                    
    return None