#include <unistd.h>             //Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <sys/ioctl.h>          //Needed for I2C port
#include <linux/i2c-dev.h>      //Needed for I2C port
#include <stdio.h>
#include <math.h>
#include "bno055.h"

const uint8_t BNO055::OFFSET_COUNT = 22;

namespace
{
const uint8_t offsets[] = {240,255,251,255,155,0,199,255,135,128,64,0,255,255,0,0,128,128,232,3,233,130};
}


BNO055::BNO055(uint8_t addr):
address(addr),
i2cFile(0)
{

}

bool BNO055::readOffsets(uint8_t *offsets)
{
    for(uint8_t i = 0; i < OFFSET_COUNT; i++)
    {
        offsets[i] = read8(static_cast<REGISTER>(ACCEL_OFFSET_X_LSB_ADDR + i));
        usleep(1000*100);
    }
}

bool BNO055::writeOffsets(const uint8_t *offsets)
{
    for(uint8_t i = 0; i < OFFSET_COUNT; i++)
    {
        write8(static_cast<REGISTER>(ACCEL_OFFSET_X_LSB_ADDR + i), offsets[i]);
        usleep(1000*10);
    }
} 

bool BNO055::begin(int file, OP_MODE mode)
{
    i2cFile = file;
    if (ioctl(i2cFile, I2C_SLAVE, address) < 0)
    {
        return false;
    }

    uint8_t id = read8(BNO055_CHIP_ID_ADDR);

    if (id != BNO055_ID)
    {
        usleep(1000*1000);
        id = read8(BNO055_CHIP_ID_ADDR);

        if (id != BNO055_ID)
        {
            return false;
        }
    }

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);

    /* Reset */
    write8(BNO055_SYS_TRIGGER_ADDR, 0x20);

    // Wait a half second for the reset to complete
    usleep(500*1000);

    while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
    {
        usleep(1000*100);
    }

    usleep(1000*50);

    /* Set to normal power mode */
    write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    usleep(1000*10);

    write8(BNO055_PAGE_ID_ADDR, 0);

    /* Set the output units */

    uint8_t unitsel = (0 << 7) | // Orientation = Android
        (0 << 4) | // Temperature = Celsius
        (0 << 2) | // Euler = Degrees
        (1 << 1) | // Gyro = Rads
        (0 << 0);  // Accelerometer = m/s^2
    write8(BNO055_UNIT_SEL_ADDR, unitsel);
     

    /* Configure axis mapping (see section 3.4) */
    
//       write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
//       usleep(1000*10);
//       write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
//       usleep(1000*10);
     

    write8(BNO055_SYS_TRIGGER_ADDR, 0x0);

    usleep(1000*10);
    
    // write calibration
    writeOffsets(offsets);

    /* Set the requested operating mode (see section 3.3) */
    setMode(mode);

    usleep(1000*20);

    return true;
}

void BNO055::setMode(OP_MODE mode)
{
    write8(BNO055_OPR_MODE_ADDR, mode);
    usleep(30*1000);
}


void BNO055::getSystemStatus(uint8_t *status, uint8_t *selfTestResults, uint8_t *systemError)
{
    if (ioctl(i2cFile, I2C_SLAVE, address) < 0)
    {
        return;
    }

    write8(BNO055_PAGE_ID_ADDR, 0);

    /* System Status (see section 4.3.58)
       ---------------------------------
       0 = Idle
       1 = System Error
       2 = Initializing Peripherals
       3 = System Iniitalization
       4 = Executing Self-Test
       5 = Sensor fusio algorithm running
       6 = System running without fusion algorithms */

    if (status != 0)
        *status = read8(BNO055_SYS_STAT_ADDR);

    /* Self Test Results (see section )
       --------------------------------
       1 = test passed, 0 = test failed
       Bit 0 = Accelerometer self test
       Bit 1 = Magnetometer self test
       Bit 2 = Gyroscope self test
       Bit 3 = MCU self test
       0x0F = all good! */

    if (selfTestResults != 0)
        *selfTestResults = read8(BNO055_SELFTEST_RESULT_ADDR);

    /* System Error (see section 4.3.59)
       ---------------------------------
       0 = No error
       1 = Peripheral initialization error
       2 = System initialization error
       3 = Self test result failed
       4 = Register map value out of range
       5 = Register map address out of range
       6 = Register map write error
       7 = BNO low power mode not available for selected operat ion mode
       8 = Accelerometer power mode not available
       9 = Fusion algorithm configuration error
       A = Sensor configuration error */

    if (systemError != 0)
        *systemError = read8(BNO055_SYS_ERR_ADDR);

    usleep(200*1000);

}

void BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
    if (ioctl(i2cFile, I2C_SLAVE, address) < 0)
    {
        return;
    }

    uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}

void BNO055::getVector(Vector *vect)
{
    if (ioctl(i2cFile, I2C_SLAVE, address) < 0)
    {
        printf ("Failed setting i2c address\n");
        return;
    }

    uint8_t buffer[6] = {0};

    /* Read vector data (6 bytes) */
    readLen(BNO055_EULER_H_LSB_ADDR, buffer, 6);

    vect->x = (float)(((int16_t)buffer[0]) | (((int16_t)buffer[1]&0x7f) << 8))/16.0f*M_PI/180.0f;
    vect->y = (float)(((int16_t)buffer[2]) | (((int16_t)buffer[3]&0x7f) << 8))/16.0f*M_PI/180.0f;
    vect->z = (float)(((int16_t)buffer[4]) | (((int16_t)buffer[5]&0x7f) << 8))/16.0f*M_PI/180.0f;
#if 0
    /* Convert the value to an appropriate range (section 3.6.4) */
    /* and assign the value to the Vector type */
    switch(vector_type)
    {
    case VECTOR_MAGNETOMETER:
        /* 1uT = 16 LSB */
        vect->x = ((double)x)/16.0;
        vect->y = ((double)y)/16.0;
        vect->z = ((double)z)/16.0;
        break;
    case VECTOR_GYROSCOPE:
        /* 1rps = 900 LSB */
        vect->x = ((double)x)/900.0;
        vect->y = ((double)y)/900.0;
        vect->z = ((double)z)/900.0;
        break;
    case VECTOR_EULER:
        /* 1 degree = 16 LSB */
        vect->x = ((double)x)/16.0;
        vect->y = ((double)y)/16.0;
        vect->z = ((double)z)/16.0;
        break;
    case VECTOR_ACCELEROMETER:
    case VECTOR_LINEARACCEL:
    case VECTOR_GRAVITY:
        /* 1m/s^2 = 100 LSB */
        vect->x = ((double)x)/100.0;
        vect->y = ((double)y)/100.0;
        vect->z = ((double)z)/100.0;
        break;
    }

    return xyz;
#endif
}

int8_t BNO055::getTemp()
{
  int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
  return temp;
}


bool BNO055::isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return false;

    return true;
}

bool BNO055::write8(REGISTER reg, uint8_t value)
{
    int8_t buff[2] = {reg, value};

    write(i2cFile, buff, sizeof(buff));

    return true;
}

uint8_t BNO055::read8(REGISTER reg )
{
    uint8_t value = 0;
    
    write(i2cFile, (char*)&reg, 1);
    usleep(50);
    read(i2cFile, (char*)&value, 1);
    return value;
}

bool BNO055::readLen(REGISTER reg,  uint8_t *buffer, uint8_t len)
{
    write(i2cFile, (char*)&reg, 1);
    usleep(100);
    read(i2cFile, buffer, len);
    return true;
}

