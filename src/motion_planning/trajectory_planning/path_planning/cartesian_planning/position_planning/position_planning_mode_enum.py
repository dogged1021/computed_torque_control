from enum import unique
from src.interface.mode_enum import ModeEnum


@unique
class PositionPlanningModeEnum(ModeEnum):
    LINE = 'line'
    ARC_CENTER = 'arc_center'
    ARC_POINT = 'arc_point'
