import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lawrence/Ballbot-MPC/ballbot_ws/install/ballbot_visualization_pkg'
