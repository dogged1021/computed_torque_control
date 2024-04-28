from enum import unique
from src.interface.mode_enum import ModeEnum


@unique
class PathPlanningModeEnum(ModeEnum):
    JOINT = 'joint'
    CARTESIAN = 'cartesian'
