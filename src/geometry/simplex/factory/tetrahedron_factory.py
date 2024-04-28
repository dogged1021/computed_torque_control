from factory.simplex_factory import SimplexFactory
from src.geometry.simplex.tetrahedron import Tetrahedron
from factory.simplex_parameter import SimplexParameter


class TetrahedronFactory(SimplexFactory):

    @property
    def key(self):
        return '4'

    def create_product(self, simplex_parameter: SimplexParameter):
        return Tetrahedron(simplex_parameter.parameter())


tetrahedron_factory = TetrahedronFactory()
tetrahedron_factory.register()
