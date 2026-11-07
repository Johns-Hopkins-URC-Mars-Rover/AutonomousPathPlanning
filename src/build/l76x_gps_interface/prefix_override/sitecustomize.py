import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/clara/AutonomousPathPlanning/src/install/l76x_gps_interface'
