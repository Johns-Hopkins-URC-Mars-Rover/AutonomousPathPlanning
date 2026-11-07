import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/clara/AutonomousPathPlanning/src/install/icm20948_imu_interface'
