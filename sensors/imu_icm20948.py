# Application code for drone interfacing with Qwiic ICM20948 IMU

import time
import sys
from qwiic_icm20948 import *
from clock_RV1805 import getTime_s

ACCELERATION_SCALING_FACTOR = 9.81/4096
GYROSCOPE_SCALING_FACTOR = 1/131
MAGNETOMETER_SCALING_FACTOR = 0.15

def initIMU(IMU):
    IMU.swReset
    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. ", file=sys.stderr)
        return
    
    IMU.sleep(False)
    IMU.lowPower(False)
    IMU.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous)
    IMU.setFullScaleRangeAccel(gpm8)
    IMU.setFullScaleRangeGyro(dps250)
    IMU.setDLPFcfgAccel(acc_d50bw4_n68bw8)
    IMU.setDLPFcfgGyro(gyr_d361bw4_n376bw5)
    IMU.enableDlpfAccel(True)
    IMU.enableDlpfGyro(True)
    IMU.startupMagnetometer()


def collectIMUData(IMU, collection_period):

    if IMU.dataReady():
        IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
        timestamp = getTime_s()

        time.sleep(collection_period)
    else:
        print("Waiting for data")
        time.sleep(0.5)

def writeIMUDataToFile(IMU, filedes):
    filedes.write('{: 10.2f}'.format(IMU.axRaw * ACCELERATION_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.ayRaw * ACCELERATION_SCALING_FACTOR) 
    + '\t' + '{: 10.2f}'.format(IMU.azRaw * ACCELERATION_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.gxRaw * GYROSCOPE_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.gyRaw * GYROSCOPE_SCALING_FACTOR) 
    + '\t' + '{: 10.2f}'.format(IMU.gzRaw * GYROSCOPE_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.mxRaw * MAGNETOMETER_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.myRaw * MAGNETOMETER_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.mzRaw * MAGNETOMETER_SCALING_FACTOR)
    + '\t')


