#include "PicoHX711.h"
#include <math.h>
#include "welfords.h"
#include "utils.h"

// const uint16_t HX711_SETTLING_TIMES_MS[] = {
//     400, // 10 SPS
//     50   // 80 SPS
// };

bool pico_hx711_is_calibration_populated(const struct PicoHX711Calibration *calib)
{
    return calib->set_offset && calib->set_slope;
}

void pico_hx711_begin(struct PicoHX711 *hx, uint8_t pin_clock, uint8_t pin_data,
                      enum PicoHX711Gain gain, enum PicoHX711Rate rate)
{
    if (mutex_is_initialized(&hx->mux)) return;
    mutex_init(&hx->mux);

    HX711_MUTEX_BLOCK(hx->mux,

                      hx->pin_clock = pin_clock;
                      hx->pin_data = pin_data;
                      hx->gain = gain;
                      hx->rate = rate;

                      gpio_init(hx->pin_clock);
                      gpio_set_dir(hx->pin_clock, GPIO_OUT);

                      gpio_init(hx->pin_data);
                      gpio_set_dir(hx->pin_data, GPIO_IN);

    );
}

bool pico_hx711_is_ready_unsafe(struct PicoHX711 *hx)
{
    return gpio_get(hx->pin_data) == false;
}

bool pico_hx711_is_ready(struct PicoHX711 *hx)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->mux,
                      res = pico_hx711_is_ready_unsafe(hx););
    return res;
}

void pico_hx711_wait_ready_unsafe(struct PicoHX711 *hx)
{
    while (!pico_hx711_is_ready_unsafe(hx))
        sleep_us(HX711_IS_READY_POLLING_DELAY_US);
}

void pico_hx711_wait_ready(struct PicoHX711 *hx)
{
    HX711_MUTEX_BLOCK(hx->mux,
                      pico_hx711_wait_ready_unsafe(hx););
}

bool pico_hx711_ready_timeout_unsafe(struct PicoHX711 *hx, uint32_t timeout_ms)
{
    absolute_time_t end_time = make_timeout_time_ms(timeout_ms);
    while (!time_reached(end_time))
    {
        if (pico_hx711_is_ready_unsafe(hx))
            return true;
        sleep_us(HX711_IS_READY_POLLING_DELAY_US);
    }
    return false;
}

bool pico_hx711_ready_timeout(struct PicoHX711 *hx, uint32_t timeout_ms)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->mux,
                      res = pico_hx711_ready_timeout_unsafe(hx, timeout_ms););
    return res;
}

void pico_hx711_power_on_unsafe(struct PicoHX711 *hx)
{
    gpio_put(hx->pin_clock, false);
}

void pico_hx711_power_on(struct PicoHX711 *hx)
{
    HX711_MUTEX_BLOCK(hx->mux,
                      pico_hx711_power_on_unsafe(hx););
}

void pico_hx711_power_off_unsafe(struct PicoHX711 *hx, bool wait_until_power_off)
{
    gpio_put(hx->pin_clock, false);
    sleep_us(HX711_PULSE_DELAY_US);
    gpio_put(hx->pin_clock, true);
    if (wait_until_power_off)
        sleep_us(HX711_POWER_DOWN_DELAY_US);
}

void pico_hx711_power_off(struct PicoHX711 *hx, bool wait_until_power_off)
{
    HX711_MUTEX_BLOCK(hx->mux,
                      pico_hx711_power_off_unsafe(hx, wait_until_power_off););
}

static inline void pico_hx711_pulse(struct PicoHX711 *hx)
{
    gpio_put(hx->pin_clock, true);
    sleep_us(HX711_PULSE_DELAY_US);
    gpio_put(hx->pin_clock, false);
    sleep_us(HX711_PULSE_DELAY_US);
}

#define HX711_MIN_VALUE INT32_C(-0x800000) // −8,388,608
#define HX711_MAX_VALUE INT32_C(0x7fffff)  // 8,388,607
static inline int32_t _hx711_get_twos_comp(const uint32_t v)
{
    return (int32_t)(-(v & +HX711_MIN_VALUE)) +
           (int32_t)(v & HX711_MAX_VALUE);
}

int32_t hx711_get_twos_comp(const uint32_t v)
{
    return _hx711_get_twos_comp(v);
}

#define HX711_READ_BITS UINT8_C(24)
bool pico_hx711_read_raw_single_unsafe(struct PicoHX711 *hx, int32_t *raw, uint32_t timeout_ms)
{
    uint32_t data = 0; // Don't forget to initialize this variable to 0!!!!!

    pico_hx711_power_on_unsafe(hx);

    if (timeout_ms > 0)
    {
        if (!pico_hx711_ready_timeout_unsafe(hx, timeout_ms))
            return false;
    }
    else
        pico_hx711_wait_ready_unsafe(hx);

    // read
    HX711_INTERRUPTS_OFF_BLOCK(

        for (uint8_t i = 0; i < HX711_READ_BITS; ++i) {
            pico_hx711_pulse(hx);
            data |= gpio_get(hx->pin_data) << ((HX711_READ_BITS - 1) - i);
        }

        // set gain
        for (uint8_t i = 0; i < hx->gain; ++i)
            pico_hx711_pulse(hx);

    );

    // to two's compliment
    *raw = _hx711_get_twos_comp(data);
    return true;
}

bool pico_hx711_read_raw_single(struct PicoHX711 *hx, int32_t *raw, uint32_t timeout_ms)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->mux,
                      res = pico_hx711_read_raw_single_unsafe(hx, raw, timeout_ms););
    return res;
}

bool pico_hx711_read_raw_stats_unsafe(struct PicoHX711 *hx, uint32_t n, float *mean,
                                      float *stdev, uint32_t *resulting_n,
                                      uint32_t timeout_ms)
{
    // It is safe to cast the int32_t to floats because the int32_t originate
    // from the hx711 and it produces numbers of 32 bits. Therefore, because
    // the float's mantissa is 24 bits wide, a number produced by the hx711
    // will always be able to be represented by a float, even if it is
    // stored in a int32_t

    // check if n == 0 or n == 1
    if (n == 0)
        return false;
    else if (n == 1)
    {
        int32_t raw;
        if (!pico_hx711_read_raw_single_unsafe(hx, &raw, timeout_ms))
            return false;
        *mean = (float)raw;
        *stdev = 0.0;
        *resulting_n = 1;
        return true;
    }

    // read values n times and use welford's online algorithm to calculate mean and stdev
    struct WelfordsAggregate agg = {};
    for (uint32_t i = 0; i < n; ++i)
    {
        int32_t raw;
        if (!pico_hx711_read_raw_single_unsafe(hx, &raw, timeout_ms))
            continue;
        welfords_update(&agg, (float)raw);
    }

    if (!welfords_finalize(&agg, mean, stdev))
        return false;
    *resulting_n = agg.count;
    return true;
}

bool pico_hx711_read_raw_stats(struct PicoHX711 *hx, uint32_t n, float *mean,
                               float *stdev, uint32_t *resulting_n, uint32_t timeout_ms)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->mux,
                      res = pico_hx711_read_raw_stats_unsafe(hx, n, mean, stdev, resulting_n, timeout_ms););
    return res;
}

#define sq(x) ((x) * (x))
bool pico_hx711_raw_to_calib(struct PicoHX711Calibration *calib, float raw_mean, float raw_stdev,
                             float *mean, float *stdev)
{
    if (!pico_hx711_is_calibration_populated(calib))
        return false;

    *mean = calib->slope * (raw_mean - calib->offset);
    float r_minus_o = raw_mean - calib->offset;
    *stdev = sqrt(sq(r_minus_o) * sq(calib->slope_e) +
                  sq(calib->slope) * (sq(raw_stdev) + sq(calib->offset_e)));

    return true;
}

bool pico_hx711_read_calib_stats_unsafe(struct PicoHX711 *hx,
                                        struct PicoHX711Calibration *calib,
                                        uint32_t n, float *mean, float *stdev,
                                        uint32_t *resulting_n, uint32_t timeout_ms)
{
    float raw_mean, raw_stdev;
    if (!pico_hx711_read_raw_stats_unsafe(hx, n, &raw_mean, &raw_stdev, resulting_n, timeout_ms))
        return false;

    return pico_hx711_raw_to_calib(calib, raw_mean, raw_stdev, mean, stdev);
}

bool pico_hx711_read_calib_stats(struct PicoHX711 *hx,
                                 struct PicoHX711Calibration *calib,
                                 uint32_t n, float *mean, float *stdev,
                                 uint32_t *resulting_n, uint32_t timeout_ms)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->mux,
                      res = pico_hx711_read_calib_stats_unsafe(hx, calib, n, mean, stdev,
                                                               resulting_n, timeout_ms););
    return res;
}

bool pico_hx711_calibrate_tare_unsafe(struct PicoHX711 *hx,
                                      struct PicoHX711Calibration *calib, uint32_t n,
                                      uint32_t *resulting_n, uint32_t timeout_ms)
{
    float raw_mean, raw_stdev;
    if (!pico_hx711_read_raw_stats_unsafe(hx, n, &raw_mean, &raw_stdev, resulting_n, timeout_ms))
        return false;

    calib->offset = raw_mean;
    calib->offset_e = raw_stdev;
    calib->set_offset = true;

    return true;
}

bool pico_hx711_calibrate_tare(struct PicoHX711 *hx,
                               struct PicoHX711Calibration *calib, uint32_t n,
                               uint32_t *resulting_n, uint32_t timeout_ms)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->mux,
                      res = pico_hx711_calibrate_tare_unsafe(hx, calib, n, resulting_n, timeout_ms););
    return res;
}

#ifdef abs
#undef abs
#endif
#define abs(x) ((x) > 0 ? (x) : -(x))
bool pico_hx711_calibrate_slope_unsafe(struct PicoHX711 *hx,
                                       struct PicoHX711Calibration *calib,
                                       uint32_t n, float weight, float weight_error,
                                       uint32_t *resulting_n, uint32_t timeout_ms)
{
    if (!calib->set_offset)
        return false;

    float raw_mean, raw_stdev;
    if (!pico_hx711_read_raw_stats_unsafe(hx, n, &raw_mean, &raw_stdev, resulting_n, timeout_ms))
        return false;

    calib->slope = weight / (raw_mean - calib->offset);

    float diff = raw_mean - calib->offset;
    float diff_sq = sq(diff);
    float body = (weight / diff_sq) * (sq(calib->offset_e) - sq(raw_stdev)) + sq(weight_error);
    body = abs(body); // this shouldn't be necessary
    calib->slope_e = sqrt(body / diff_sq);
    calib->set_slope = true;

    return true;
}

bool pico_hx711_calibrate_slope(struct PicoHX711 *hx,
                                struct PicoHX711Calibration *calib,
                                uint32_t n, float weight, float weight_error,
                                uint32_t *resulting_n, uint32_t timeout_ms)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->mux,
                      res = pico_hx711_calibrate_slope_unsafe(hx, calib, n, weight, weight_error,
                                                              resulting_n, timeout_ms););
    return res;
}