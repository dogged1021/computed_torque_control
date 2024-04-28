from abc import ABC, abstractmethod
from interface.mode_enum import ModeEnum


class Register(ABC):

    @classmethod
    @abstractmethod
    def mode(cls) -> ModeEnum:
        pass
