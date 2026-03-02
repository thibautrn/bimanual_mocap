import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/asurite.ad.asu.edu/troisin/Documents/robot/bimanual_mocap/bimanual_ws/install/bimanualrobot_bringup'
