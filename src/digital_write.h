
#pragma once

#ifndef DIGITAL_WRITE_H
#define DIGITAL_WRITE_H

#include <inttypes.h>
#include <Arduino.h>
#include "hc595.h"

HC595 hc595{23, 25, 24, 2};

void pin_mode(uint8_t pin, uint8_t mode)
{
    if (pin > 29)
    {
    }
    else
    {
        pinMode(pin, mode);
    }
}

void digital_write(uint8_t pin, uint8_t l)
{
    if (pin > 29)
    {
        uint8_t t = pin - 30;

        hc595.write_bit(l, t % 8, t / 8);
    }
    else
    {
        digitalWrite(pin, l);
    }
}

#endif DIGITAL_WRITE_H