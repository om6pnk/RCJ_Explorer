#ifndef _IR_sensor_H_
#define _IR_sensor_H_

#include "Arduino.h"
#include "Wire.h"

//----------------------------------------------------------------

class IR_sensor
{
public:
    void init(int address);
    void update(int);
    int angle;
    int distance;
    short data[2];
    bool ret;

private:
    int address_;
};

#endif // _IR_sensor_H_