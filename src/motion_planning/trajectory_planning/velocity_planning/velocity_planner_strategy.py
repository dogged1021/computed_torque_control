from abc import ABC

from src.interface.strategy import Strategy, Parameter

from src.interface.parameter import Parameter

class VelocityPlannerStrategy(Strategy, ABC):

    def __init__(self, parameter: Parameter):
        super().__init__(parameter)
