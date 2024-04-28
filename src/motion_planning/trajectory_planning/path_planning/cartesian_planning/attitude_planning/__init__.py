import sys
import os

# 获取当前脚本文件所在目录的绝对路径
current_path = os.path.abspath(os.path.dirname(__file__))

# 将上级目录路径添加到搜索路径
parent_path = os.path.abspath(os.path.join(current_path, os.pardir))
sys.path.append(parent_path)

# 将本级目录路径添加到搜索路径
sys.path.append(current_path)


from attitude_parameter import AttitudeParameter
from attitude_planner import AttitudePlanner
from attitude_planner_strategy import AttitudePlannerStrategy

from one_attitude_planning import *
from two_attitude_planning import *
from three_attitude_planning import *
