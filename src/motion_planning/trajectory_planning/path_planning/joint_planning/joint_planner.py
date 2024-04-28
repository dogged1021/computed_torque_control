from src.interface.mode_enum import ModeEnum
from path_planning.path_planner_strategy import PathPlannerStrategy

from path_planning.path_planning_mode_enum import PathPlanningModeEnum

from joint_planning.joint_parameter import JointParameter


class JointPlanner(PathPlannerStrategy):

    def __init__(self, parameter: JointParameter):
        super().__init__(parameter)

        self.plan()

    @classmethod
    def mode(cls) -> ModeEnum:
        return PathPlanningModeEnum.JOINT

    def plan(self):
        pass

    def interpolate(self, s: float):
        return self.parameter.get_q0() + s * (self.parameter.get_q1() - self.parameter.get_q0())
