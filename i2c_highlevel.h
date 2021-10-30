#ifndef _I2C_HIGHLEVEL_H_
#define _I2C_HIGHLEVEL_H_

#include "i2c_lowlevel.h"
#include "i2c_sensors.h"

class HighLevelI2C
{
public:
    bool timings(struct timing_t &tm);
    void resetTimings(void);

    HighLevelI2C(PinName sda, PinName scl, int addr);
    bool write(uint8_t reg, uint8_t val, int len);
    bool read(uint8_t reg, int len);
    uint32_t get(void);
    bool loop(void);
    bool ack(void);
    bool error(void);
    bool recover(void);
    
private:
    LowLevelI2C i2c;
    Timer timer;
    int i2c_state;
    uint32_t i2c_val;
    uint8_t i2c_reg;
    uint8_t i2c_addr;
    bool i2c_error;
    bool i2c_ack;
    struct timing_t max_state_duration;
};

#endif
