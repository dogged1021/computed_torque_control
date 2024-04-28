from abc import ABC, abstractmethod
from typing import Type

from interface.parameter import Parameter
from interface.strategy import Strategy
from interface.factory import Factory


class StrategyWrapper(ABC):
    def __init__(self, parameter: Parameter) -> None:
        self.strategy: Strategy = self.get_factory().strategy.get(parameter.get_mode())(parameter)

    @staticmethod
    def get_factory() -> Type[Factory]:
        return Factory
