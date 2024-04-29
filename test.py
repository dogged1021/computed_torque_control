from tests.test_computed_torque_controller import TestComputedTorqueController
from tests.test_pid_controller import TestPIDControl

# # 创建测试类实例
# test_instance = TestComputedTorqueController()

# # # 调用PID控制测试函数
# # test_instance.test_pid_control()

# # # 调用计算扭矩控制测试函数
# test_instance.test_computed_torque_control()

test_instance = TestPIDControl()

# 调用PID控制测试函数
test_instance.test_pid_control()
# test_instance.test_robot_move()