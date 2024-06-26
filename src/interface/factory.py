from typing import Type

from interface.mode_enum import ModeEnum
from interface.register import Register


class Factory:
    strategy = {}

    @classmethod
    def get_strategy_by_mode(cls, mode: ModeEnum) -> Type[Register]:
        return cls.strategy.get(mode)

    @classmethod
    def register(cls, mode: ModeEnum, strategy: Type[Register]) -> None:
        cls.strategy[mode] = strategy
