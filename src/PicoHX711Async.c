#include "PicoHX711Async.h"
#include "utils.h"

void pico_hx711_async_begin(struct PicoHX711Async *hx, uint8_t pin_clock, uint8_t pin_data,
                            enum PicoHX711Gain gain, enum PicoHX711Rate rate)
{
    pico_hx711_begin(&hx->hx, pin_clock, pin_data, gain, rate);
}

bool pico_hx711_async_is_ready_unsafe(struct PicoHX711Async *hx)
{
    return pico_hx711_is_ready_unsafe(&hx->hx);
}

bool pico_hx711_async_is_ready(struct PicoHX711Async *hx)
{
    return pico_hx711_is_ready(&hx->hx);
}

void pico_hx711_async_wait_ready_unsafe(struct PicoHX711Async *hx)
{
    pico_hx711_wait_ready_unsafe(&hx->hx);
}

void pico_hx711_async_wait_ready(struct PicoHX711Async *hx)
{
    pico_hx711_wait_ready(&hx->hx);
}

bool pico_hx711_async_ready_timeout_unsafe(struct PicoHX711Async *hx, uint32_t timeout_ms)
{
    return pico_hx711_ready_timeout_unsafe(&hx->hx, timeout_ms);
}

bool pico_hx711_async_ready_timeout(struct PicoHX711Async *hx, uint32_t timeout_ms)
{
    return pico_hx711_ready_timeout(&hx->hx, timeout_ms);
}

void pico_hx711_async_power_on_unsafe(struct PicoHX711Async *hx)
{
    pico_hx711_power_on_unsafe(&hx->hx);
}

void pico_hx711_async_power_on(struct PicoHX711Async *hx)
{
    pico_hx711_power_on(&hx->hx);
}

void pico_hx711_async_power_off_unsafe(struct PicoHX711Async *hx, bool wait_until_power_off)
{
    pico_hx711_power_off_unsafe(&hx->hx, wait_until_power_off);
}

void pico_hx711_async_power_off(struct PicoHX711Async *hx, bool wait_until_power_off)
{
    pico_hx711_power_off(&hx->hx, wait_until_power_off);
}

bool pico_hx711_async_read_raw_single_unsafe(struct PicoHX711Async *hx, int32_t *raw, uint32_t timeout_ms)
{
    return pico_hx711_read_raw_single_unsafe(&hx->hx, raw, timeout_ms);
}

bool pico_hx711_async_read_raw_single(struct PicoHX711Async *hx, int32_t *raw, uint32_t timeout_ms)
{
    return pico_hx711_read_raw_single(&hx->hx, raw, timeout_ms);
}

bool pico_hx711_async_calibrate_tare_blocking_unsafe(struct PicoHX711Async *hx,
                                                     struct PicoHX711Calibration *calib, uint32_t n,
                                                     uint32_t *resulting_n, uint32_t timeout_ms)
{
    return pico_hx711_calibrate_tare_unsafe(&hx->hx, calib, n, resulting_n, timeout_ms);
}

bool pico_hx711_async_calibrate_tare_blocking(struct PicoHX711Async *hx,
                                              struct PicoHX711Calibration *calib, uint32_t n,
                                              uint32_t *resulting_n, uint32_t timeout_ms)
{
    return pico_hx711_calibrate_tare(&hx->hx, calib, n, resulting_n, timeout_ms);
}

bool pico_hx711_async_calibrate_slope_blocking_unsafe(struct PicoHX711Async *hx,
                                                      struct PicoHX711Calibration *calib,
                                                      uint32_t n, float weight, float weight_error,
                                                      uint32_t *resulting_n, uint32_t timeout_ms)
{
    return pico_hx711_calibrate_slope_unsafe(&hx->hx, calib, n, weight, weight_error, resulting_n, timeout_ms);
}
bool pico_hx711_async_calibrate_slope_blocking(struct PicoHX711Async *hx,
                                               struct PicoHX711Calibration *calib,
                                               uint32_t n, float weight, float weight_error,
                                               uint32_t *resulting_n, uint32_t timeout_ms)
{
    return pico_hx711_calibrate_slope(&hx->hx, calib, n, weight, weight_error, resulting_n, timeout_ms);
}

// async funcs

bool pico_hx711_async_end_read_unsafe(struct PicoHX711Async *hx,
    struct PicoHX711Calibration *calib,
    float *mean, float *stdev, uint32_t *resulting_n)
    {
        float raw_mean, raw_stdev;
        if (!welfords_finalize(&hx->agg, &raw_mean, &raw_stdev))
            return false;
        *resulting_n = hx->agg.count;

        // reset aggregate
        hx->agg.mean = 0.0;
        hx->agg.M2 = 0.0;
        hx->agg.count = 0;

        return pico_hx711_raw_to_calib(calib, raw_mean, raw_stdev, mean, stdev);
    }

bool pico_hx711_async_read_unsafe(struct PicoHX711Async *hx, uint32_t timeout_ms)
{
    int32_t raw;
    bool res = pico_hx711_async_read_raw_single_unsafe(hx, &raw, timeout_ms);
    if (!res) return false;
    welfords_update(&hx->agg, (float)raw);
    return true;
}

bool pico_hx711_async_read(struct PicoHX711Async *hx, uint32_t timeout_ms)
{
    bool res;
    HX711_MUTEX_BLOCK(hx->hx.mux,
        res = pico_hx711_async_read_unsafe(hx, timeout_ms);
    );
    return res;
}