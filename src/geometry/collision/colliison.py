from simplex.interface.support import Support
from collision.GJK import GJK


class Collision:

    @staticmethod
    def is_collision(shape0: Support, shape1: Support) -> bool:
        return GJK.is_intersecting(shape0, shape1)
