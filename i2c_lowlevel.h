#ifndef _I2C_LOWLEVEL_H_
#define _I2C_LOWLEVEL_H_

#include "mbed.h"

class LowLevelI2C
{
public:
    LowLevelI2C(PinName sda, PinName scl);
    void stop(void);
    void start(void);
    bool write(uint8_t val);
    uint8_t read(bool send_ack);
    bool recover(void);
    bool ready(void);
    
protected:
    DigitalInOut pin_sda;
    DigitalInOut pin_scl;
    bool scl_input;
    bool sda_input;
    bool i2c_ack;
    uint8_t i2c_value;
    int command;
    int step;
    
private:
    void delay(void);
    void setSCL(void);
    void setSDA(void);
    void clearSCL(void);
    void clearSDA(void);
    int getSCL(void);
    int getSDA(void);
};

#endif
