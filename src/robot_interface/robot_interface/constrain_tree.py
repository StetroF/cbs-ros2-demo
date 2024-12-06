#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com



'''

from robot_interface.constraints import Constraints

from typing import Dict
import numpy as np

# from .agent import Agent

class CTNode:

    def __init__(self, constraints: Constraints,
                       solution: Dict[str, np.ndarray]):

        self.constraints = constraints
        self.solution = solution
        self.cost = self.sic(solution)

    # Sum-of-Individual-Costs heuristics
    @staticmethod
    def sic(solution):
        ###solution是类似于[('P7', 3.14), ('P6', 7.64), ('P5', 14.39)]，[-1][1]就是当前path到达最后一个点的预期耗时
        return sum(sol[-1][1] for sol in solution.values()) 


    def __lt__(self, other):
        return self.cost < other.cost

    def __str__(self):
        return str(self.constraints.agent_constraints)

