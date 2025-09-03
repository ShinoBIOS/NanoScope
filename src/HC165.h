
#pragma once

#ifndef HC165_H
#define HC165_H

#include <inttypes.h>
#include <Arduino.h>
#include "digital_write.h"

class HC165
{
    uint8_t data_p;
    uint8_t latch_p;
    uint8_t clk_p;

    uint8_t shift_in(pin_size_t dataPin, pin_size_t clockPin, BitOrder bitOrder)
    {
        uint8_t value = 0;
        uint8_t i;

        for (i = 0; i < 8; ++i)
        {
            digital_write(clockPin, HIGH);

            if (bitOrder == LSBFIRST)
            {
                value |= digitalRead(dataPin) << i;
            }
            else
            {
                value |= digitalRead(dataPin) << (7 - i);
            }

            digital_write(clockPin, LOW);
        }

        return value;
    }

public:
    HC165(uint8_t data_p, uint8_t latch_p, uint8_t clk_p) : data_p(data_p), latch_p(latch_p), clk_p(clk_p)
    {
        pin_mode(clk_p, OUTPUT);
        pin_mode(latch_p, OUTPUT);
        pin_mode(data_p, INPUT);
        digital_write(latch_p, HIGH);
    }

    uint8_t get_byte()
    {
        digital_write(latch_p, LOW); // щелкнули защелкой
        digital_write(latch_p, HIGH);
        return shift_in(data_p, clk_p, MSBFIRST); // считали данные
    }
};

#endif