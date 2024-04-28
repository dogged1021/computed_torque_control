import sys
import os

# 获取当前脚本文件所在目录的绝对路径
current_path = os.path.abspath(os.path.dirname(__file__))

# 将上级目录路径添加到搜索路径
parent_path = os.path.abspath(os.path.join(current_path, os.pardir))
sys.path.append(parent_path)

# 将本级目录路径添加到搜索路径
sys.path.append(current_path)


# from simplex_factory_pool import SimplexFactoryPool
# from simplex_parameter import SimplexParameter
# from simplex_factory import SimplexFactory
# from point_factory import PointFactory
# from line_segment_factory import LineSegmentFactory
# from triangle_factory import TriangleFactory
# from tetrahedron_factory import TetrahedronFactory
