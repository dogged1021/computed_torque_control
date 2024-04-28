from src.geometry.simplex.simplex import Simplex
from factory.simplex_factory_interface import SimplexFactoryInterface
from factory.simplex_parameter import SimplexParameter


class SimplexFactoryPool:
    factory_pool: dict[str, SimplexFactoryInterface] = {}

    @classmethod
    def create_product(cls, simplex_parameter: SimplexParameter) -> Simplex:
        return cls.factory_pool[simplex_parameter.key].create_product(simplex_parameter)
