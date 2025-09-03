
#pragma once

#ifndef HC4053D_H
#define HC4053D_H

#include <inttypes.h>
#include <Arduino.h>
#include "digital_write.h"

enum HC4053D_CHS
{
    HC4053D_CH_A,
    HC4053D_CH_B,
    HC4053D_CH_C,
};

class HC4053D
{
    struct
    {
        uint8_t p, l;
    } chs[3];

public:
    HC4053D(uint8_t asw, uint8_t bsw, uint8_t ssw) : chs{{asw, 0}, {bsw, 0}, {ssw, 0}}
    {
        if (asw)
            pin_mode(asw, OUTPUT);
        if (bsw)
            pin_mode(bsw, OUTPUT);
        if (ssw)
            pin_mode(ssw, OUTPUT);
    }

    void set_channel_level(uint8_t ch, uint8_t l)
    {
        if (chs[ch].p)
        {
            digital_write(chs[ch].p, l);
            chs[ch].l = l;
        }
        else
        {
            Serial.printf("HC4053D: error channel %d not avalible", ch);
        }
    }

    uint8_t get_channel_level(uint8_t ch)
    {
        return chs[ch].l;
    }
};

#endif HC4053D_H