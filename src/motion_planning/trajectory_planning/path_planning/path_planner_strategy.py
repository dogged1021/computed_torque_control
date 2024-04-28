from abc import ABC

from src.interface.strategy import Strategy

from path_planning.path_parameter import PathParameter


class PathPlannerStrategy(Strategy, ABC):

    def __init__(self, parameter: PathParameter):
        super().__init__(parameter)
