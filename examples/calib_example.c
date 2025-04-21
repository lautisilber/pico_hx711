#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "PicoHX711.h"

#include <ctype.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#define HX711_SCK 16
#define HX711_DT 17
#define HX711_MULT_1 5
#define HX711_MULT_2 6
#define HX711_MULT_3 7
#define HX711_MULT_4 4
#define N_MULT 4

#define USER_INPUT_BUFFER_SIZE 32

bool read_line_blocking(char *buf, size_t buf_size);
bool read_line_float_blocking(char *buf, size_t buf_size, float *f);
void reboot();

int main()
{
    stdio_init_all();

    sleep_ms(1000);
    printf("hx711 demo init\n");

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
    struct PicoHX711Calibration calib = {};
    pico_hx711_begin(&hx, HX711_SCK, HX711_DT, A128, SPS_10);

    pico_hx711_power_on(&hx);

    printf("Remove all weight from scale and press enter...");

    while (getchar() != '\n')
    {
        sleep_ms(10);
    }

    printf("\nCalibrating tare...");

    const uint32_t n = 200;
    uint32_t resulting_n;
    bool res = pico_hx711_calibrate_tare(&hx, &calib, n, &resulting_n, 1000);
    if (!res)
    {
        printf(" Error calibrating tare!\n");
        reboot();
    }
    printf(" Calibrated tare! (resulting_n = %lu)\n", resulting_n);

    printf("Place the test object on the scale and enter the weight of the test object... ");

    char buf[USER_INPUT_BUFFER_SIZE] = {};
    float weight, weight_error;

    if (!read_line_float_blocking(buf, USER_INPUT_BUFFER_SIZE, &weight))
    {
        printf("\nCouldn't read input weight\n");
        reboot();
    }

    printf("%f", weight);
    printf("\nEnter the error of the weight of the test object... ");

    if (!read_line_float_blocking(buf, USER_INPUT_BUFFER_SIZE, &weight_error))
    {
        printf("\nCouldn't read input weight error\n");
        reboot();
    }
    
    printf("%f", weight_error);
    printf("\nCalibrating slope... ");

    res = pico_hx711_calibrate_slope(&hx, &calib, n, weight, weight_error, &resulting_n, 1000);
    if (!res)
    {
        printf("Error calibrating slope!\n");
        reboot();
    }
    printf("\nCalibrated slope! (resulting_n = %lu)\n", resulting_n);

    printf("Calibration is the following\n");
    printf("\toffset: %.10e +/- %.10e\n", calib.offset, calib.offset_e);
    printf("\tslope: %.10e +/- %.10e\n", calib.slope, calib.slope_e);
    printf("\nNow you can weigh stuff with a calibrated scale\n");

    for (;;)
    {
        float mean, stdev;
        res = pico_hx711_read_calib_stats(&hx, &calib, 20, &mean, &stdev, &resulting_n, 1000);
        printf("read: (%s) weight: %f, error: %f (n_stats: %lu)\n",
               (res ? "true" : "false"), mean, stdev, resulting_n);
    }
}

bool read_line_blocking(char *buf, size_t buf_size)
{
    size_t i;
    bool res = false;
    for (i = 0; i < buf_size-1; ++i)
    {
        char c = getchar();
        if (c == '\n')
        {
            res = true;
            break;
        }
        buf[i] = c;
    }
    buf[i] = '\0';
    return res;
}

bool read_line_float_blocking(char *buf, size_t buf_size, float *f)
{
    memset(buf, 0, buf_size);
    bool res = read_line_blocking(buf, buf_size);
    if (!res)
        return false;

    char *endptr;
    *f = strtof(buf, &endptr);

    return (buf != endptr) && (errno != ERANGE);
}

void reboot()
{
    watchdog_reboot(0, 0, 0);
}