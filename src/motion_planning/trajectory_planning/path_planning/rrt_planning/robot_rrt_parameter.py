import copy
from typing import Union, Iterable

import numpy as np

from rrt_planning.rrt_map import RRTMap
from rrt_planning.i_check_collision import ICheckCollision

from rrt_planning.rrt_parameter import RRTParameter
from rrt_planning.check_collision_robot import CheckCollisionRobot
from src.robot.robot import Robot


class RobotRRTParameter(RRTParameter):
    def __init__(self, start: Union[np.ndarray, Iterable], goal: Union[np.ndarray, Iterable], robot: Robot,
                 expand_dis: float = 1.0, goal_sample_rate: float = 10.0, max_iter: int = 100, radius: float = 10.0,
                 animation: bool = False) -> None:
        super().__init__(start, goal, expand_dis, goal_sample_rate, max_iter, radius, animation)
        self.__robot = copy.deepcopy(robot)

    @property
    def robot(self):
        return copy.deepcopy(self.__robot)

    def create_check_collision(self, rrt_map: RRTMap) -> ICheckCollision:
        return CheckCollisionRobot(rrt_map.obstacles, self.expand_dis, self.robot)
