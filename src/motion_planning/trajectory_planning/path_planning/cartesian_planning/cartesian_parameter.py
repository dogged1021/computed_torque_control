from src.interface.mode_enum import ModeEnum
from path_planning.path_parameter import PathParameter
from path_planning.path_planning_mode_enum import PathPlanningModeEnum

from position_planning.position_parameter import PositionParameter
from attitude_planning.attitude_parameter import AttitudeParameter


class CartesianParameter(PathParameter):

    def __init__(self, position_parameter: PositionParameter, attitude_parameter: AttitudeParameter) -> None:
        super().__init__()

        self.position_parameter = position_parameter
        self.attitude_parameter = attitude_parameter

    def get_position_parameter(self):
        return self.position_parameter

    def get_attitude_parameter(self):
        return self.attitude_parameter

    def get_length(self):
        return self.position_parameter.get_length()

    @classmethod
    def get_mode(cls) -> ModeEnum:
        return PathPlanningModeEnum.CARTESIAN
