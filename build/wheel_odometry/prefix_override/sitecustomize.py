import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gianfi/Documents/KvaserCar_outdoor_test/install/wheel_odometry'
