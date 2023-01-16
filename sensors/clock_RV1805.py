import time
import qwiic_i2c

RTC_I2C_ADDRESS = 0x69

def initRTC():
    if qwiic_i2c.isDeviceConnected(RTC_I2C_ADDRESS) == False:
        print("The RV-1805 RTC is not connected to the system")
        return


def getTime_s():
    i2cDriver = qwiic_i2c.getI2CDriver()
    data = i2cDriver.readBlock(RTC_I2C_ADDRESS, 0x0, 4)
    hundredths = (data[0] & 0b1111) + 10 * ((data[0] >> 4) & 0b1111)
    seconds = (data[1] & 0b1111) + 10 * ((data[1] >> 4) & 0b111)
    minutes = (data[2] & 0b1111) + 10 * ((data[2] >> 4) & 0b111)
    hours = (data[3] & 0b1111) + 10 * ((data[3] >> 4) & 0b11)
    return hundredths * 0.01 + seconds + minutes * 60 + hours * 3600