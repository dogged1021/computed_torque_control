import sys
import os

# 获取当前脚本文件所在目录的绝对路径
current_path = os.path.abspath(os.path.dirname(__file__))

# 将上级目录路径添加到搜索路径
parent_path = os.path.abspath(os.path.join(current_path, os.pardir))
sys.path.append(parent_path)

# 将本级目录路径添加到搜索路径
sys.path.append(current_path)


from src.geometry.simplex.geometry import Geometry
from src.geometry.simplex.simplex import Simplex
from src.geometry.simplex.point import Point
from src.geometry.simplex.vector import Vector
from src.geometry.simplex.unit_vector import UnitVector
from src.geometry.simplex.line import Line
from src.geometry.simplex.line_segment import LineSegment
from src.geometry.simplex.triangle import Triangle
from src.geometry.simplex.tetrahedron import Tetrahedron
from src.geometry.simplex.interface import *
from src.geometry.simplex.factory import *

import factory
import interface