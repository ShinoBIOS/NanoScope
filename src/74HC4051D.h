
#pragma once

#ifndef HC4051D_H
#define HC4051D_H

#include <inttypes.h>
#include <Arduino.h>
#include "digital_write.h"

class HC4051D
{
    uint8_t a;
    uint8_t b;
    uint8_t c;

public:
    HC4051D(uint8_t a, uint8_t b, uint8_t c) : a(a), b(b), c(c)
    {
        if (a)
            pin_mode(a, OUTPUT);
        if (b)
            pin_mode(b, OUTPUT);
        if (c)
            pin_mode(c, OUTPUT);
    }

    void set_channel(uint8_t ch)
    {
        if (ch < 8)
        {
            digital_write(a, bitRead(ch, 0));
            digital_write(b, bitRead(ch, 1));
            digital_write(c, bitRead(ch, 2));
        }
        else
        {
            Serial.printf("HC4051D: error channel %d not avalible", ch);
        }
    }
};

#endif HC4051sD_H