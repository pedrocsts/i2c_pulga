#include "mbed.h"
#include "i2c_sensors.h"
#include "i2c_highlevel.h"

extern Serial pc;

static Timer timer;

enum SensorI2CType {
    Pos2k5Pa,
    Pos5kPa,
    Pos10kPa,
    Pos20kPa,
    Pos40kPa,
    Pos100kPa,
    Pos200kPa,
    Pos500kPa,
    Pos700kPa,
    Pos1000kPa,
};

static HighLevelI2C sensor1(P1_6, P0_2, 0x6d); // sda1, scl1
static HighLevelI2C sensor2(P1_10, P0_28, 0x6d); // sda2, scl2

static float pressure1 = 0.0;
static float pressure2 = 0.0;

static int numMeas = 0;
static int numOK = 0;
static int duration = 0;

static enum SensorI2CType sensor_type1 = Pos10kPa;
static enum SensorI2CType sensor_type2 = Pos700kPa;

static void convert(const enum SensorI2CType type, float &pressure);

static bool sensor_error = false;

enum SensorStep {
    SENSOR_STEP0,
    SENSOR_STEP1,
    SENSOR_STEP2,
    SENSOR_STEP3,
    SENSOR_STEP4,
    SENSOR_STEP5,
};

static enum SensorStep sensorStep = SENSOR_STEP0;

bool I2c_GetComTimings(struct timing_t &tm)
{
    struct timing_t tm1, tm2;
    
    bool ok1 = sensor1.timings(tm1);
    bool ok2 = sensor2.timings(tm2);
    
    if (ok1 && ok2)
    {
        if (tm1.duration_us > tm2.duration_us) {
            tm = tm1;
        }
        else {
            tm = tm2;
        }
    }
    else if (ok1)
    {
        tm = tm1;
    }
    else if (ok2)
    {
        tm = tm2;
    }
    else {
        return false;
    }
    return true;
}

void I2c_GetMeasStats(int &error, int &total)
{
    error = numMeas - numOK;
    total = numMeas;
}

int I2c_GetMeasDuration(void)
{
    return duration;
}

void I2c_SensorLoop(void)
{
    int ret;
    
    bool busy1 = sensor1.loop();
    bool busy2 = sensor2.loop();
    
    switch(sensorStep)
    {
    case SENSOR_STEP0:
        numMeas++;
        timer.stop();
        timer.reset();
        timer.start();
        sensor1.read(0xA5, 16);
        sensor2.read(0xA5, 16);
        sensorStep = SENSOR_STEP1;
        break;
        
    case SENSOR_STEP1:
        if (!busy1 && !busy2)
        {
            if (sensor1.error() || sensor2.error())
            {
                sensor_error = true;
                sensorStep = SENSOR_STEP0;
            }
            else
            {
                sensor1.write(0xA5, sensor1.get() & 0x7fd, 16);
                sensor2.write(0xA5, sensor2.get() & 0x7fd, 16);
                sensorStep = SENSOR_STEP2;
            }
        }
        break;
    
    case SENSOR_STEP2:
        if (!busy1 && !busy2)
        {
            if (sensor1.error() || sensor2.error())
            {
                sensor_error = true;
                sensorStep = SENSOR_STEP0;
            }
            else
            {
                sensor1.write(0x30, 0x0A, 8);
                sensor2.write(0x30, 0x0A, 8);
                sensorStep = SENSOR_STEP3;
            }
        }
        break;
        
    case SENSOR_STEP3:
        if (!busy1 && !busy2)
        {
            if (sensor1.error() || sensor2.error())
            {
                sensor_error = true;
                sensorStep = SENSOR_STEP0;
            }
            else
            {
                sensor1.read(0x30, 8);
                sensor2.read(0x30, 8);
                sensorStep = SENSOR_STEP4;
            }
        }
        break;
    
    case SENSOR_STEP4:
        if (!busy1 && !busy2)
        {
            if (sensor1.error() || sensor2.error())
            {
                sensor_error = true;
                sensorStep = SENSOR_STEP0;
            }
            else if ((sensor1.get() & 0x08) || (sensor2.get() & 0x08)) {
                sensorStep = SENSOR_STEP3;
            }
            else
            {
                sensor1.read(0x06, 24);
                sensor2.read(0x06, 24);
                sensorStep = SENSOR_STEP5;
            }
        }
        break;
    
    case SENSOR_STEP5:
        if (!busy1 && !busy2)
        {
            if (sensor1.error() || sensor2.error()) {
                sensor_error = true;
            }
            else
            {
                int32_t p1_aux = (int32_t)sensor1.get();
                int32_t p2_aux = (int32_t)sensor2.get();

                if (p1_aux >= 0x0800000) {
                    p1_aux -= 0x1000000;
                }
                if (p2_aux >= 0x0800000) {
                    p2_aux -= 0x1000000;
                }

                pressure1 = p1_aux;
                pressure2 = p2_aux;

                convert(sensor_type1, pressure1);
                convert(sensor_type2, pressure2);
                
                sensor_error = false;
                numOK++;
                timer.stop();
                if (timer.read_us() > duration) {
                    duration = timer.read_us();
                }
            }
            sensorStep = SENSOR_STEP0;
        }
        break;
    }
}

void I2c_SensorSetup(void)
{
    printf("I2C Sensor Initialization, please wait...\r\n");
    
    sensor1.recover();
    sensor2.recover();
    
    sensorStep = SENSOR_STEP0;
    sensor_error = false;
    
    pressure1 = 0.0;
    pressure2 = 0.0;
    
    sensor_type1 = Pos10kPa;
    sensor_type2 = Pos700kPa;
    
    for(int retry = 0; retry < 10; retry++)
    {
        if (retry != 0) {
            printf("I2C Sensor Initialization Error. Retry...\r\n");
        }
        
        do {
            I2c_SensorLoop();
        } while(sensorStep != SENSOR_STEP0);
        
        if (!sensor_error) {
            break;
        }
    }
    
    if (sensor_error) {
        printf("I2C Sensor Initialization Failed.\r\n");
    }
    else {
        printf("I2C Sensor Initialization Succeeded.\r\n");
    }
}

bool I2c_Read_Pressure(float &pressure)
{
    pressure = pressure1;
    return true;
}

bool I2c_Read_O2(float &pressure)
{
    pressure = pressure2;
    return true;
}

bool I2c_SensorError(void)
{
    return sensor_error;
}

static void convert(const enum SensorI2CType type, float &pressure)
{
    // calculate the pressure in kPa
    switch(type)
    {
    case Pos2k5Pa: // (2e23 / 2e12) = 2e11
        pressure /= 2048.0;
        break;

    case Pos5kPa: // (2e23 / 2e13) = 2e10
        pressure /= 1024.0;
        break;

    case Pos10kPa: // (2e23 / 2e14) = 2e9
        pressure /= 512.0;
        break;

    case Pos20kPa: // (2e23 / 2e15) = 2e8
        pressure /= 256.0;
        break;

    case Pos40kPa:
        pressure /= 128.0;
        break;

    case Pos100kPa:
        pressure /= 64.0;
        break;

    case Pos200kPa:
        pressure /= 32.0;
        break;

    case Pos500kPa:
        pressure /= 16.0;
        break;

    case Pos700kPa:
    case Pos1000kPa:
        pressure /= 8.0;
        break;
    }
    pressure /= 1000.0;
}
