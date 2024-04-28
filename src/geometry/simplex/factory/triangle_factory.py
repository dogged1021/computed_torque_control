from factory.simplex_factory import SimplexFactory
from src.geometry.simplex.triangle import Triangle
from factory.simplex_parameter import SimplexParameter


class TriangleFactory(SimplexFactory):

    @property
    def key(self):
        return '3'

    def create_product(self, simplex_parameter: SimplexParameter):
        return Triangle(simplex_parameter.parameter())


triangle_factory = TriangleFactory()
triangle_factory.register()
