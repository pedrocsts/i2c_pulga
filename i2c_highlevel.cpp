#include "i2c_highlevel.h"

enum {
    STATE_I2C_IDLE = 0,
    STATE_I2C_WRITE8_START,
    STATE_I2C_WRITE8_STOP,
    STATE_I2C_WRITE8_ADDR,
    STATE_I2C_WRITE8_REG,
    STATE_I2C_WRITE8_VAL,
    STATE_I2C_WRITE16_START,
    STATE_I2C_WRITE16_STOP,
    STATE_I2C_WRITE16_ADDR,
    STATE_I2C_WRITE16_REG,
    STATE_I2C_WRITE16_VAL_MSB,
    STATE_I2C_WRITE16_VAL_LSB,
    STATE_I2C_READ8_START,
    STATE_I2C_READ8_STOP,
    STATE_I2C_READ8_ADDR,
    STATE_I2C_READ8_REG,
    STATE_I2C_READ8_RESTART,
    STATE_I2C_READ8_ADDR2,
    STATE_I2C_READ8_VAL,
    STATE_I2C_READ16_START,
    STATE_I2C_READ16_STOP,
    STATE_I2C_READ16_ADDR,
    STATE_I2C_READ16_REG,
    STATE_I2C_READ16_RESTART,
    STATE_I2C_READ16_ADDR2,
    STATE_I2C_READ16_VAL_MSB,
    STATE_I2C_READ16_VAL_LSB,
    STATE_I2C_READ24_START,
    STATE_I2C_READ24_STOP,
    STATE_I2C_READ24_ADDR,
    STATE_I2C_READ24_REG,
    STATE_I2C_READ24_RESTART,
    STATE_I2C_READ24_ADDR2,
    STATE_I2C_READ24_VAL_MSB,
    STATE_I2C_READ24_VAL_CSB,
    STATE_I2C_READ24_VAL_LSB,
};

struct StateName {
    int state;
    const char *name;
};

#define STATE_NAME_ENTRY(x) {.state = x, .name = #x}
#define STATE_NAME_ENTRY_SENTINEL {.name = NULL}

static StateName stateNames[] = {
    STATE_NAME_ENTRY(STATE_I2C_IDLE),
    STATE_NAME_ENTRY(STATE_I2C_WRITE8_START),
    STATE_NAME_ENTRY(STATE_I2C_WRITE8_STOP),
    STATE_NAME_ENTRY(STATE_I2C_WRITE8_ADDR),
    STATE_NAME_ENTRY(STATE_I2C_WRITE8_REG),
    STATE_NAME_ENTRY(STATE_I2C_WRITE8_VAL),
    STATE_NAME_ENTRY(STATE_I2C_WRITE16_START),
    STATE_NAME_ENTRY(STATE_I2C_WRITE16_STOP),
    STATE_NAME_ENTRY(STATE_I2C_WRITE16_ADDR),
    STATE_NAME_ENTRY(STATE_I2C_WRITE16_REG),
    STATE_NAME_ENTRY(STATE_I2C_WRITE16_VAL_MSB),
    STATE_NAME_ENTRY(STATE_I2C_WRITE16_VAL_LSB),
    STATE_NAME_ENTRY(STATE_I2C_READ8_START),
    STATE_NAME_ENTRY(STATE_I2C_READ8_STOP),
    STATE_NAME_ENTRY(STATE_I2C_READ8_ADDR),
    STATE_NAME_ENTRY(STATE_I2C_READ8_REG),
    STATE_NAME_ENTRY(STATE_I2C_READ8_RESTART),
    STATE_NAME_ENTRY(STATE_I2C_READ8_ADDR2),
    STATE_NAME_ENTRY(STATE_I2C_READ8_VAL),
    STATE_NAME_ENTRY(STATE_I2C_READ16_START),
    STATE_NAME_ENTRY(STATE_I2C_READ16_STOP),
    STATE_NAME_ENTRY(STATE_I2C_READ16_ADDR),
    STATE_NAME_ENTRY(STATE_I2C_READ16_REG),
    STATE_NAME_ENTRY(STATE_I2C_READ16_RESTART),
    STATE_NAME_ENTRY(STATE_I2C_READ16_ADDR2),
    STATE_NAME_ENTRY(STATE_I2C_READ16_VAL_MSB),
    STATE_NAME_ENTRY(STATE_I2C_READ16_VAL_LSB),
    STATE_NAME_ENTRY(STATE_I2C_READ24_START),
    STATE_NAME_ENTRY(STATE_I2C_READ24_STOP),
    STATE_NAME_ENTRY(STATE_I2C_READ24_ADDR),
    STATE_NAME_ENTRY(STATE_I2C_READ24_REG),
    STATE_NAME_ENTRY(STATE_I2C_READ24_RESTART),
    STATE_NAME_ENTRY(STATE_I2C_READ24_ADDR2),
    STATE_NAME_ENTRY(STATE_I2C_READ24_VAL_MSB),
    STATE_NAME_ENTRY(STATE_I2C_READ24_VAL_CSB),
    STATE_NAME_ENTRY(STATE_I2C_READ24_VAL_LSB),
    STATE_NAME_ENTRY_SENTINEL,
};

void HighLevelI2C::resetTimings(void)
{
    max_state_duration.state = 0;
    max_state_duration.duration_us = 0;
    max_state_duration.state_name = NULL;
}

bool HighLevelI2C::timings(struct timing_t &tm)
{
    int i = 0;
    
    while(stateNames[i].name != NULL)
    {
        if (stateNames[i].state == max_state_duration.state)
        {
            tm = max_state_duration;
            tm.state_name = stateNames[i].name;
            return true;
        }
        i++;
    }
    
    tm.duration_us = 0;
    tm.state = -1;
    tm.state_name = NULL;
    return false;
}

HighLevelI2C::HighLevelI2C(PinName sda, PinName scl, int addr) : i2c(sda, scl)
{
    i2c_addr  = (uint8_t)((addr << 1) & 0xFE);
    i2c_val   = 0x0;
    i2c_reg   = 0x0;
    i2c_error = false;
    i2c_ack   = false;
    i2c_state = STATE_I2C_IDLE;
    resetTimings();
}

bool HighLevelI2C::read(uint8_t reg, int len)
{
    if (i2c_state != STATE_I2C_IDLE) {
        return false;
    }
    switch (len)
    {
    case 8:
        i2c_state = STATE_I2C_READ8_START;
        break;
        
    case 16:
        i2c_state = STATE_I2C_READ16_START;
        break;
        
    case 24:
        i2c_state = STATE_I2C_READ24_START;
        break;
        
    default:
        return false;
    }
    i2c_val   = 0x0;
    i2c_reg   = reg;
    i2c_error = false;
    i2c_ack   = false;
    return true;
}

bool HighLevelI2C::write(uint8_t reg, uint8_t val, int len)
{
    if (i2c_state != STATE_I2C_IDLE) {
        return false;
    }
    switch (len)
    {
    case 8:
        i2c_state = STATE_I2C_WRITE8_START;
        break;
        
    case 16:
        i2c_state = STATE_I2C_WRITE16_START;
        break;
        
    default:
        return false;
    }
    i2c_val   = val;
    i2c_reg   = reg;
    i2c_error = false;
    i2c_ack   = false;
    return true;
}

uint32_t HighLevelI2C::get(void)
{
    return i2c_val;
}

bool HighLevelI2C::ack(void)
{
    return i2c_ack;
}

bool HighLevelI2C::error(void)
{
    return i2c_error;
}

bool HighLevelI2C::recover(void)
{
    return i2c.recover();
}

bool HighLevelI2C::loop(void)
{
    int old_state = i2c_state;
    uint32_t aux;
    bool ret;
    
    timer.reset();
    timer.start();
    
    switch(i2c_state)
    {
    case STATE_I2C_WRITE8_START:
        i2c.start();
        i2c_state = STATE_I2C_WRITE8_ADDR;
        break;
        
    case STATE_I2C_WRITE8_STOP:
        i2c.stop();
        i2c_state = STATE_I2C_IDLE;
        break;
        
    case STATE_I2C_WRITE8_ADDR:
        ret = i2c.write(i2c_addr);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_WRITE8_REG;
            }
            else {
                i2c_state = STATE_I2C_WRITE8_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_WRITE8_REG:
        ret = i2c.write(i2c_reg);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_WRITE8_VAL;
            }
            else {
                i2c_state = STATE_I2C_WRITE8_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_WRITE8_VAL:
        ret = i2c.write(i2c_val & 0xFF);
        if (i2c.ready())
        {
            i2c_ack = ret;
            
            if (!i2c_ack) {
                i2c_error = true;
            }
            
            i2c_state = STATE_I2C_WRITE8_STOP;
        }
        break;
        
    case STATE_I2C_WRITE16_START:
        i2c.start();
        i2c_state = STATE_I2C_WRITE16_ADDR;
        break;
        
    case STATE_I2C_WRITE16_STOP:
        i2c.stop();
        i2c_state = STATE_I2C_IDLE;
        break;
        
    case STATE_I2C_WRITE16_ADDR:
        ret = i2c.write(i2c_addr);
        if (i2c.ready())
        {
            if (ret){
                i2c_state = STATE_I2C_WRITE16_REG;
            }
            else {
                i2c_state = STATE_I2C_WRITE16_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_WRITE16_REG:
        ret = i2c.write(i2c_reg);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_WRITE16_VAL_MSB;
            }
            else {
                i2c_state = STATE_I2C_WRITE16_STOP;
                i2c_error = true;
            }
        }
        break;
    
    case STATE_I2C_WRITE16_VAL_MSB:
        ret = i2c.write((i2c_val >> 8) & 0xFF);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_WRITE16_VAL_LSB;
            }
            else {
                i2c_state = STATE_I2C_WRITE16_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_WRITE16_VAL_LSB:
        ret = i2c.write(i2c_val & 0xFF);
        if (i2c.ready())
        {
            i2c_ack = ret;
            
            if (!i2c_ack) {
                i2c_error = true;
            }
            
            i2c_state = STATE_I2C_WRITE16_STOP;
        }
        break;
    
    case STATE_I2C_READ8_START:
        i2c.start();
        i2c_state = STATE_I2C_READ8_ADDR;
        break;
        
    case STATE_I2C_READ8_STOP:
        i2c.stop();
        i2c_state = STATE_I2C_IDLE;
        break;
        
    case STATE_I2C_READ8_ADDR:
        ret = i2c.write(i2c_addr);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ8_REG;
            }
            else {
                i2c_state = STATE_I2C_READ8_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ8_REG:
        ret = i2c.write(i2c_reg);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ8_RESTART;
            }
            else {
                i2c_state = STATE_I2C_READ8_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ8_RESTART:
        i2c.start();
        i2c_state = STATE_I2C_READ8_ADDR2;
        break;
        
    case STATE_I2C_READ8_ADDR2:
        ret = i2c.write(i2c_addr | 0x01);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ8_VAL;
            }
            else {
                i2c_state = STATE_I2C_READ8_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ8_VAL:
        aux = ((uint32_t)(i2c.read(false))) & 0xFF;
        if (i2c.ready())
        {
            i2c_val = aux;
            i2c_state = STATE_I2C_READ8_STOP;
        }
        break;
    
    case STATE_I2C_READ16_START:
        i2c.start();
        i2c_state = STATE_I2C_READ16_ADDR;
        break;
        
    case STATE_I2C_READ16_STOP:
        i2c.stop();
        i2c_state = STATE_I2C_IDLE;
        break;
        
    case STATE_I2C_READ16_ADDR:
        ret = i2c.write(i2c_addr);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ16_REG;
            }
            else {
                i2c_state = STATE_I2C_READ16_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ16_REG:
        ret = i2c.write(i2c_reg);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ16_RESTART;
            }
            else {
                i2c_state = STATE_I2C_READ16_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ16_RESTART:
        i2c.start();
        i2c_state = STATE_I2C_READ16_ADDR2;
        break;
        
    case STATE_I2C_READ16_ADDR2:
        ret = i2c.write(i2c_addr | 0x01);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ16_VAL_MSB;
            }
            else {
                i2c_state = STATE_I2C_READ16_STOP;
                i2c_error = true;
            }
        }
        break;
    
    case STATE_I2C_READ16_VAL_MSB:
        aux = ((uint32_t)(i2c.read(true))) & 0xFF;
        if (i2c.ready())
        {
            i2c_val = (aux << 8) & 0xFF00;
            i2c_state = STATE_I2C_READ16_VAL_LSB;
        }
        break;
    
    case STATE_I2C_READ16_VAL_LSB:
        aux = ((uint32_t)(i2c.read(false))) & 0xFF;
        if (i2c.ready())
        {
            i2c_val |= aux & 0xFF;
            i2c_state = STATE_I2C_READ16_STOP;
        }
        break;
        
    case STATE_I2C_READ24_START:
        i2c.start();
        i2c_state = STATE_I2C_READ24_ADDR;
        break;
        
    case STATE_I2C_READ24_STOP:
        i2c.stop();
        i2c_state = STATE_I2C_IDLE;
        break;
        
    case STATE_I2C_READ24_ADDR:
        ret = i2c.write(i2c_addr);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ24_REG;
            }
            else {
                i2c_state = STATE_I2C_READ24_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ24_REG:
        ret = i2c.write(i2c_reg);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ24_RESTART;
            }
            else {
                i2c_state = STATE_I2C_READ24_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ24_RESTART:
        i2c.start();
        i2c_state = STATE_I2C_READ24_ADDR2;
        break;
        
    case STATE_I2C_READ24_ADDR2:
        ret = i2c.write(i2c_addr | 0x01);
        if (i2c.ready())
        {
            if (ret) {
                i2c_state = STATE_I2C_READ24_VAL_MSB;
            }
            else {
                i2c_state = STATE_I2C_READ24_STOP;
                i2c_error = true;
            }
        }
        break;
        
    case STATE_I2C_READ24_VAL_MSB:
        aux = ((uint32_t)(i2c.read(true))) & 0xFF;
        if (i2c.ready())
        {
            i2c_val = (aux << 16) & 0xFF0000;
            i2c_state = STATE_I2C_READ24_VAL_CSB;
        }
        break;
    
    case STATE_I2C_READ24_VAL_CSB:
        aux = ((uint32_t)(i2c.read(true))) & 0xFF;
        if (i2c.ready())
        {
            i2c_val |= (aux << 8) & 0xFF00;
            i2c_state = STATE_I2C_READ24_VAL_LSB;
        }
        break;
    
    case STATE_I2C_READ24_VAL_LSB:
        aux = ((uint32_t)(i2c.read(false))) & 0xFF;
        if (i2c.ready())
        {
            i2c_val |= aux;
            i2c_state = STATE_I2C_READ24_STOP;
        }
        break;
    }
    
    timer.stop();
    
    if (timer.read_us() > max_state_duration.duration_us)
    {
        max_state_duration.duration_us = timer.read_us();
        max_state_duration.state = old_state;
    }
    
    return (i2c_state != STATE_I2C_IDLE);
}
