from abc import abstractmethod

from factory.simplex_factory_pool import SimplexFactoryPool
from factory.simplex_factory_interface import SimplexFactoryInterface


class SimplexFactory(SimplexFactoryInterface):

    @property
    @abstractmethod
    def key(self):
        pass

    def register(self):
        SimplexFactoryPool.factory_pool[self.key] = self
