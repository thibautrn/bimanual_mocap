import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thibaut/Documents/bimanual_mocap/install/bimanualrobot_bringup'
