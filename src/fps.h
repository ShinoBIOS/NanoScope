
#pragma once

#ifndef FPS_H
#define FPS_H

#include <inttypes.h>
#include <pico/time.h>

class FPS
{
    uint32_t fps_counter = 0;
    uint32_t fps_timer;

public:
    FPS()
    {
        fps_counter = 0;
        fps_timer = to_ms_since_boot(get_absolute_time());
    }

    uint32_t tick(uint32_t time)
    {
        if (time - fps_timer > 1000)
        {  
            uint32_t fps = fps_counter;
            fps_counter = 0;
            fps_timer = time;
            return fps;
        }
        else
        {
            ++fps_counter;
        }

        return 0;
    }
};

#endif