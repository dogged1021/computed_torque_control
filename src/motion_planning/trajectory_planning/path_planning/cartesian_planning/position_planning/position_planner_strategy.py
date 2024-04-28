from abc import ABC, abstractmethod

import numpy as np

from src.interface.strategy import Strategy

from position_planning.position_parameter import PositionParameter


class PositionPlannerStrategy(Strategy, ABC):

    def __init__(self, parameter: PositionParameter):
        super().__init__(parameter)

    @abstractmethod
    def plan(self) -> None:
        pass

    @abstractmethod
    def interpolate(self, s) -> np.ndarray:
        pass
