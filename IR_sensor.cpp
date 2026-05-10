#include "IR_sensor.h"
// Initial IR sensor
void IR_sensor::init(int address)
{
    address_ = address;
    Wire.begin();
}

// Update IR sensor
void IR_sensor::update(int g)
{
    Wire.requestFrom(address_, 2 * sizeof(short));

    short buffer[2 * sizeof(short)];
    for (unsigned int i = 0; i < 2 * sizeof(short); i++)
    {
        buffer[i] = Wire.read();
    }
    memcpy(&data, buffer, 2 * sizeof(short));

    angle = data[0];
    distance = data[1];
    ret = (data[0] > 400);

    Serial.print(" Ball: ");
    Serial.print(angle);
    Serial.print(" ");
    Serial.print(ret);
    Serial.print(" ");
    Serial.println(distance);
}