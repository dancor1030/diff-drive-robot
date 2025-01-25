import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/daniel/robotica/ros2_ws_2402/install/modelrobot_pkg_gbb'
