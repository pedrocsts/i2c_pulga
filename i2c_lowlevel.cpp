#include "i2c_lowlevel.h"

enum {
    CMD_IDLE = 0,
    CMD_WRITE,
    CMD_READ,
};

LowLevelI2C::LowLevelI2C(PinName sda, PinName scl) : pin_sda(sda), pin_scl(scl)
{
    pin_scl.input();
    pin_scl.mode(PullNone);
    pin_sda.input();
    pin_sda.mode(PullNone);
    scl_input = true;
    sda_input = true;
    command = CMD_IDLE;
    step = 0;
}

bool LowLevelI2C::ready(void)
{
    return (command == CMD_IDLE);
}

bool LowLevelI2C::write(uint8_t val)
{
    if (command == CMD_IDLE)
    {
        command = CMD_WRITE;
        i2c_value = val;
        i2c_ack = false;
        step = 0;
    }
    else if (step < 8)
    {
        if (i2c_value & 0x80) {
            setSDA();
        }
        else {
            clearSDA();
        }
        delay();
        setSCL();
        delay();
        clearSCL();
        i2c_value <<= 1;
        step++;
    }
    else
    {
        setSDA();
        delay();
        setSCL();
        delay();
        i2c_ack = (getSDA() != 1);
        clearSCL();
        command = CMD_IDLE;
    }
    return i2c_ack;
}

uint8_t LowLevelI2C::read(bool send_ack)
{
    if (command == CMD_IDLE)
    {
        command = CMD_READ;
        i2c_value = 0x0;
        i2c_ack = send_ack;
        step = 0;
        setSDA();
    }
    else if (step < 8)
    {
        i2c_value <<= 1;
        delay();
        setSCL();
        delay();
        
        if (getSDA() == 1) {
            i2c_value |= 0x01;
        }
        else {
            i2c_value &= 0xFE;
        }
        clearSCL();
        step++;
    }
    else
    {
        if (i2c_ack) {
            clearSDA();
        }
        else {
            setSDA();
        }
        delay();
        setSCL();
        delay();
        clearSCL();
        command = CMD_IDLE;
    }
    return i2c_value;
}

void LowLevelI2C::stop(void)
{
    clearSDA();
    delay();
    setSCL();
    delay();
    setSDA();
    delay();
}

void LowLevelI2C::start(void)
{
    setSCL();
    setSDA();
    delay();
    clearSDA();
    delay();
    clearSCL();
}

void LowLevelI2C::delay(void)
{
    wait_ns(1250);
}

void LowLevelI2C::setSCL(void)
{
    if (!scl_input) {
        pin_scl.input();
    }
    scl_input = true;
}

void LowLevelI2C::setSDA(void)
{
    if (!sda_input) {
        pin_sda.input();
    }
    sda_input = true;
}

void LowLevelI2C::clearSCL(void)
{
    if (scl_input)
    {
        pin_scl.output();
        pin_scl = 0;
    }
    scl_input = false;
}

void LowLevelI2C::clearSDA(void)
{
    if (sda_input)
    {
        pin_sda.output();
        pin_sda = 0;
    }
    sda_input = false;
}

int LowLevelI2C::getSCL(void)
{
    setSCL();
    return pin_scl;
}

int LowLevelI2C::getSDA(void)
{
    setSDA();
    return pin_sda;
}

bool LowLevelI2C::recover(void)
{
    setSCL();
    setSDA();
    delay();

    // Return as SCL is low and no access to become master.
    if(getSCL() == 0) {
        return false;
    }
    // Return successfully as SDA and SCL is high
    if(getSDA() == 1) {
        return true;
    }
    // Send clock pulses, for device to recover
    for(int i = 0; i < 10; i++)
    {
        clearSCL();
        delay();
        setSCL();
        delay();
    }
    stop();
    
    if ((getSCL() == 0) || (getSDA() == 0)) {
        // Return as SCL is low and no access to become master.
        return false;
    }
    return true;
}
