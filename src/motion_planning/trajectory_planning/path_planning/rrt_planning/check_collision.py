import copy
from typing import Union, List, Tuple
import multiprocessing

from src.geometry.simplex.line_segment import LineSegment
from src.geometry.collision.colliison import Collision

from rrt_planning.i_check_collision import ICheckCollision


class CheckCollision(ICheckCollision):

    def __init__(self, obstacles: Union[List, Tuple]) -> None:
        self._obstacles = copy.deepcopy(obstacles)

    def check_collision(self, line_segment: LineSegment, pool: multiprocessing.Pool = None):
        for obstacle in self._obstacles:
            if Collision.is_collision(line_segment, obstacle):
                return True
        return False
