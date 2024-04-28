import sys
import os

# 获取当前脚本文件所在目录的绝对路径
current_path = os.path.abspath(os.path.dirname(__file__))

# 将上级目录路径添加到搜索路径
parent_path = os.path.abspath(os.path.join(current_path, os.pardir))
sys.path.append(parent_path)

# 将本级目录路径添加到搜索路径
sys.path.append(current_path)

import path_planning.blend_planning
import cartesian_planning
import joint_planning
import rrt_planning

from path_parameter import PathParameter
from path_planner_strategy import PathPlannerStrategy
from path_planner import PathPlanner

from joint_planning import *
from cartesian_planning import *
from rrt_planning import *
from blend_planning import *
