# Application code for drone interfacing with Qwiic ICM20948 IMU

import time
import sys
import qwiic_icm20948
from clock_RV1805 import getTime_s

ACCELERATION_SCALING_FACTOR = 9.81/16348
GYROSCOPE_SCALING_FACTOR = 1/131
MAGNETOMETER_SCALING_FACTOR = 0.15

def initIMU(IMU):

    print(str(IMU.address))
    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. ", file=sys.stderr)
        return
    IMU.begin()

def collectIMUData(IMU, collection_period):

    if IMU.dataReady():
        IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
        timestamp = getTime_s()

        print(\
         '{: 06d}'.format(IMU.axRaw)\
        , '\t', '{: 06d}'.format(IMU.ayRaw)\
        , '\t', '{: 06d}'.format(IMU.azRaw)\
        , '\t', '{: 06d}'.format(IMU.gxRaw)\
        , '\t', '{: 06d}'.format(IMU.gyRaw)\
        , '\t', '{: 06d}'.format(IMU.gzRaw)\
        , '\t', '{: 06d}'.format(IMU.mxRaw)\
        , '\t', '{: 06d}'.format(IMU.myRaw)\
        , '\t', '{: 06d}'.format(IMU.mzRaw)\
        )

        time.sleep(collection_period)
    else:
        print("Waiting for data")
        time.sleep(0.5)

def writeIMUDataToFile(IMU, filedes):
    filedes.write('{: 10.2f}'.format(IMU.axRaw * GYROSCOPE_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.ayRaw * GYROSCOPE_SCALING_FACTOR) 
    + '\t' + '{: 10.2f}'.format(IMU.azRaw * GYROSCOPE_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.gxRaw * ACCELERATION_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.gyRaw * ACCELERATION_SCALING_FACTOR) 
    + '\t' + '{: 10.2f}'.format(IMU.gzRaw * ACCELERATION_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.mxRaw * MAGNETOMETER_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.myRaw * MAGNETOMETER_SCALING_FACTOR)
    + '\t' + '{: 10.2f}'.format(IMU.mzRaw * MAGNETOMETER_SCALING_FACTOR)
    + '\t')


