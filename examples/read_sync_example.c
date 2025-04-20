/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "PicoHX711.h"

#define HX711_SCK 16
#define HX711_DT 17
#define HX711_MULT_1 5
#define HX711_MULT_2 6
#define HX711_MULT_3 7
#define HX711_MULT_4 4
#define N_MULT 4

int main()
{
    stdio_init_all();

    sleep_ms(1000);
    printf("hx711 demo init");

    const uint8_t mult_pins[N_MULT] = {
        HX711_MULT_1, HX711_MULT_2, HX711_MULT_3, HX711_MULT_4};
    for (uint8_t i = 0; i < N_MULT; ++i)
    {
        uint8_t p = mult_pins[i];
        gpio_init(p);
        gpio_set_dir(p, GPIO_OUT);
        gpio_put(p, false);
    }

    struct PicoHX711 hx = {};
    pico_hx711_begin(&hx, HX711_SCK, HX711_DT, A128, SPS_10);

    pico_hx711_power_on(&hx);

    int32_t raw;
    bool s;
    for (;;)
    {
        printf("hx711 raw: ");
        s = pico_hx711_read_raw_single(&hx, &raw, 500);
        printf("(%s) %li\n", (s ? "true" : "false"), raw);
        // sleep_ms(1000);
    }
}