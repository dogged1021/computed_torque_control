import sys
import os

# 获取当前脚本文件所在目录的绝对路径
current_path = os.path.abspath(os.path.dirname(__file__))

# 将上级目录路径添加到搜索路径
parent_path = os.path.abspath(os.path.join(current_path, os.pardir))
sys.path.append(parent_path)

# 将本级目录路径添加到搜索路径
sys.path.append(current_path)


import path_planning
import velocity_planning

from path_planning import *
from velocity_planning import *

from trajectory_parameter import TrajectoryParameter
from trajectory_planner import TrajectoryPlanner
