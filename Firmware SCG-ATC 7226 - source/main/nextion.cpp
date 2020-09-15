#include "include/nextion.h"
#include <Arduino.h>

Nextion::Nextion() {}

void Nextion::setupScreen(Stream &port)
{
    device = &port;
    while (!device);
    //need this if screen is new and default baud rate is set to 9600
    device->print("bauds=115200");

    this->flush();

    this->setDim(100);
}

void Nextion::setPage(const String page)
{
    device->print("page");
    device->write(0x20);
    device->print(page);
    this->flush();
}

void Nextion::setText(const String name, const String value)
{
    device->print(name);
    device->print(".txt=");
    device->write(0x22);
    device->print(value);
    device->write(0x22);
    this->flush();
}

void Nextion::setNumber(const String name, const float value)
{
    device->print(name);
    device->print(".txt=");
    device->write(0x22);
    device->print(value);
    device->write(0x22);
    this->flush();
}

void Nextion::setVal(const String name, const String value)
{
    device->print(name);
    device->print(".val=");
    device->print(value);
    this->flush();
}

void Nextion::setPic(const String name, const String value)
{
    device->print(name);
    device->print(".pic=");
    device->print(value);
    this->flush();
}

void Nextion::setVisibility(const String name, const String value)
{
    device->print("vis");
    device->write(0x20);
    device->print(name);
    device->write(0x2C);
    device->print(value);
    this->flush();
}

void Nextion::setDim(const int dim)
{
    device->print("dims=");
    device->print(dim);
    this->flush();
}

void Nextion::flush()
{
    device->write(0xFF);
    device->write(0xFF);
    device->write(0xFF);
    device->flush();
}

// -------------------------Addons------------------------------------

float Nextion::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Nextion::mapInt(float x, float in_min, float in_max, int out_min, int out_max)
{
    return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
