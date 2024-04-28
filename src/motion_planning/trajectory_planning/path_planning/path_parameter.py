from abc import ABC

from src.interface.parameter import Parameter


class PathParameter(Parameter, ABC):
    def get_length(self):
        pass
