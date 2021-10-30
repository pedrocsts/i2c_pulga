#ifndef _I2C_SENSORS_H_
#define _I2C_SENSORS_H_

struct timing_t
{
    int duration_us;
    int state;
    const char *state_name;
};

extern void I2c_SensorSetup(void);

extern bool I2c_Read_Pressure(float &pressure);

extern bool I2c_Read_O2(float &pressure);

extern void I2c_SensorLoop(void);

extern bool I2c_SensorError(void);

extern bool I2c_GetComTimings(struct timing_t &tm);

extern void I2c_GetMeasStats(int &error, int &total);

extern int I2c_GetMeasDuration(void);

#endif
