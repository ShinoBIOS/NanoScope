
#pragma once

#ifndef HC595_H
#define HC595_H

#include <inttypes.h>
#include <Arduino.h>

class HC595
{
    uint8_t dp;
    uint8_t cp;
    uint8_t lp;
    uint8_t *lastBytes;
    uint8_t count;

public:
    HC595(uint8_t dp, uint8_t cp, uint8_t lp, uint8_t count) : dp(dp), cp(cp), lp(lp), count(count)
    {
        pinMode(lp, OUTPUT);
        pinMode(cp, OUTPUT);
        pinMode(dp, OUTPUT);
        digitalWrite(lp, HIGH);

        lastBytes = new uint8_t[count]{0};
        write_value(0, count);
    }

    ~HC595()
    {
        delete lastBytes;
    }

    void write_value(uint8_t byte, uint8_t c)
    {
        lastBytes[c] = byte;
        digitalWrite(lp, LOW); // начинаем передачу данных

        for (int i = count - 1; i >= 0; i--)
        {
            uint8_t byte = lastBytes[i];
            shiftOut(dp, cp, LSBFIRST, byte);
        }

        digitalWrite(lp, HIGH); // прекращаем передачу данных
    }

    void write_bit(uint8_t bit, uint8_t pos, uint8_t c)
    {
        bitWrite(lastBytes[c], 7 - pos, bit);
        write_value(lastBytes[c], c);
    }
};

#endif HC595_H