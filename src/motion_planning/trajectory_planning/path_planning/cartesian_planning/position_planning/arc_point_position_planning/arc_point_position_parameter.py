import numpy as np

from src.interface.mode_enum import ModeEnum
from position_planning.position_parameter import PositionParameter
from position_planning.position_planning_mode_enum import PositionPlanningModeEnum


class ArcPointPositionParameter(PositionParameter):

    def __init__(self, t0: np.ndarray, t1: np.ndarray, tc: np.ndarray):
        super().__init__()

        self.t0 = t0
        self.t1 = t1
        self.tc = tc

    @classmethod
    def get_mode(cls) -> ModeEnum:
        return PositionPlanningModeEnum.ARC_POINT

    def get_t0(self) -> np.ndarray:
        return self.t0

    def get_t1(self) -> np.ndarray:
        return self.t1

    def get_tc(self) -> np.ndarray:
        return self.tc

    def get_length(self):
        super().get_length()
