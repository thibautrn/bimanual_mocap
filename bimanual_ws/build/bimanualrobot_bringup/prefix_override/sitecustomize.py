import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/install/bimanualrobot_bringup'
