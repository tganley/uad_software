import sys

sys.path.append('sensors')

from imu_icm20948 import *
from clock_RV1805 import *

IMU_COLLECTION_PERIOD = 0.4


def system_init():
    initIMU()

def drone_loop():
    while(True):
        collectIMUData(IMU_COLLECTION_PERIOD)

if __name__ == '__main__':
    try:
        system_init()
        drone_loop()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)