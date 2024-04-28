from enum import unique
from src.interface.mode_enum import ModeEnum


@unique
class AttitudePlannerModeEnum(ModeEnum):
    ONE = 'one'
    TWO = 'two'
    THREE = 'three'
