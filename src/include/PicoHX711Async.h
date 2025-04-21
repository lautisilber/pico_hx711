#ifndef _PICO_HX711_ASYNC_H_
#define _PICO_HX711_ASYNC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "PicoHX711.h"
#include "welfords.h"

    struct PicoHX711Async
    {
        struct PicoHX711 hx;
        struct WelfordsAggregate agg;
    };

    extern void pico_hx711_async_begin(struct PicoHX711Async *hx, uint8_t pin_clock, uint8_t pin_data,
                                       enum PicoHX711Gain gain, enum PicoHX711Rate rate);

    extern bool pico_hx711_async_is_ready_unsafe(struct PicoHX711Async *hx);
    extern bool pico_hx711_async_is_ready(struct PicoHX711Async *hx);
    extern void pico_hx711_async_wait_ready_unsafe(struct PicoHX711Async *hx);
    extern void pico_hx711_async_wait_ready(struct PicoHX711Async *hx);
    extern bool pico_hx711_async_ready_timeout_unsafe(struct PicoHX711Async *hx, uint32_t timeout_ms);
    extern bool pico_hx711_async_ready_timeout(struct PicoHX711Async *hx, uint32_t timeout_ms);

    extern void pico_hx711_async_power_on_unsafe(struct PicoHX711Async *hx);
    extern void pico_hx711_async_power_on(struct PicoHX711Async *hx);
    extern void pico_hx711_async_power_off_unsafe(struct PicoHX711Async *hx, bool wait_until_power_off);
    extern void pico_hx711_async_power_off(struct PicoHX711Async *hx, bool wait_until_power_off);

    extern bool pico_hx711_async_read_raw_single_unsafe(struct PicoHX711Async *hx, int32_t *raw, uint32_t timeout_ms);
    extern bool pico_hx711_async_read_raw_single(struct PicoHX711Async *hx, int32_t *raw, uint32_t timeout_ms);

    extern bool pico_hx711_async_calibrate_tare_blocking_unsafe(struct PicoHX711Async *hx,
                                                                struct PicoHX711Calibration *calib, uint32_t n,
                                                                uint32_t *resulting_n, uint32_t timeout_ms);
    extern bool pico_hx711_async_calibrate_tare_blocking(struct PicoHX711Async *hx,
                                                         struct PicoHX711Calibration *calib, uint32_t n,
                                                         uint32_t *resulting_n, uint32_t timeout_ms);

    extern bool pico_hx711_async_calibrate_slope_blocking_unsafe(struct PicoHX711Async *hx,
                                                                 struct PicoHX711Calibration *calib,
                                                                 uint32_t n, float weight, float weight_error,
                                                                 uint32_t *resulting_n, uint32_t timeout_ms);
    extern bool pico_hx711_async_calibrate_slope_blocking(struct PicoHX711Async *hx,
                                                          struct PicoHX711Calibration *calib,
                                                          uint32_t n, float weight, float weight_error,
                                                          uint32_t *resulting_n, uint32_t timeout_ms);

    // truly async funcs
    extern bool pico_hx711_async_end_read_unsafe(struct PicoHX711Async *hx,
                                                 struct PicoHX711Calibration *calib,
                                                 float *mean, float *stdev, uint32_t *resulting_n);
    extern bool pico_hx711_async_read_unsafe(struct PicoHX711Async *hx, uint32_t timeout_ms);
    extern bool pico_hx711_async_read(struct PicoHX711Async *hx, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* _PICO_HX711_ASYNC_H_ */