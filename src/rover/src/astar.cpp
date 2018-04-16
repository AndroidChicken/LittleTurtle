#include <unistd.h>             //Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <sys/ioctl.h>          //Needed for I2C port
#include <linux/i2c-dev.h>      //Needed for I2C port
#include <string.h>
#include "astar.h"

namespace
{
#pragma pack(1)
struct MotorMessage
{
    unsigned char reg;
    char leftSpeed;
    char rightSpeed;
};
#pragma pack()

const int addr = 0x14;          
}


//*****************************************************************************
// 
//*****************************************************************************
AStar::AStar()
{

}

//*****************************************************************************
// 
//*****************************************************************************
AStar::~AStar()
{
}

//*****************************************************************************
// 
//*****************************************************************************
void AStar::readBuffer(int i2cFile, char reg, char *buff, int size)
{
    write(i2cFile, &reg, 1);

    usleep(50);

    for (int i = 0; i < size; i++)
    {
        read(i2cFile, buff, 1);
        buff++;
    }
}


//*****************************************************************************
// 
//*****************************************************************************
bool AStar::init()
{
    return true;
}

//*****************************************************************************
// Read the status registers which includes battery voltage and left/right pos
//*****************************************************************************
bool  AStar::readState(int i2cFile, AStar::State &state)
{
    bool rc = false;
    if (ioctl(i2cFile, I2C_SLAVE, addr) >= 0)
    {
        readBuffer(i2cFile, 0, (char*)&state, sizeof(state));
        rc = true;
    }

    return rc;
}

//*****************************************************************************
// Send a velocity command to the motors
//*****************************************************************************
bool AStar::setMotor(int i2cFile, char left, char right)
{
    bool rc = false;
    if (ioctl(i2cFile, I2C_SLAVE, addr) >= 0)
    {
        MotorMessage msg = {10, left, right};
        write(i2cFile, &msg, sizeof(msg)); 
        rc = true;
    }

    return rc;
}
