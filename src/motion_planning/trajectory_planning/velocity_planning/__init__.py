import sys
import os

# 获取当前脚本文件所在目录的绝对路径
current_path = os.path.abspath(os.path.dirname(__file__))

# 将上级目录路径添加到搜索路径
parent_path = os.path.abspath(os.path.join(current_path, os.pardir))
sys.path.append(parent_path)

# 将本级目录路径添加到搜索路径
sys.path.append(current_path)

from velocity_planning_mode_enum import VelocityPlanningModeEnum
from velocity_parameter import VelocityParameter
from velocity_planner import VelocityPlanner
from velocity_planner_strategy import VelocityPlannerStrategy
from cubic_velocity_planning import *
from quintic_velocity_planning import *
