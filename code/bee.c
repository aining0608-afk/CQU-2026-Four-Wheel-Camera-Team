/*
 * bee.c
 *
 *  Created on: 2023Äê3ÔÂ5ÈÕ
 *      Author: 688
 */
#include "bee.h"


uint8 bee_time = 0;



void bee_init(void)
{
    gpio_init(P33_10, GPO,0, GPO_PUSH_PULL);
}

void bee(void)
{
    if(bee_time)
    {
        bee_time--;
        gpio_set_level(P33_10, 1);
    }
    else
    {
        gpio_set_level(P33_10, 0);
    }
}

