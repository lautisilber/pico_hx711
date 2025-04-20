#ifndef _PICO_HX711_H_
#define _PICO_HX711_H_

#include "pico/stdlib.h"
#include "pico/sync.h"

#define HX711_PULSE_DELAY_US 1
#define HX711_IS_READY_POLLING_DELAY_US 1000
#define HX711_POWER_DOWN_DELAY_US 60

#ifdef __cplusplus
extern "C"
{
#endif

    enum PicoHX711Gain
    {
        A128 = 1,
        B32 = 2,
        A64 = 3
    };

    enum PicoHX711Rate
    {
        SPS_10,
        SPS_80
    };

    struct PicoHX711
    {
        uint8_t pin_clock;
        uint8_t pin_data;
        enum PicoHX711Gain gain;
        enum PicoHX711Rate rate;

        // internal
        mutex_t mux;
        critical_section_t cs;
    };

    extern void pico_hx711_begin(struct PicoHX711 *hx, uint8_t pin_clock, uint8_t pin_data,
                                 enum PicoHX711Gain gain, enum PicoHX711Rate rate);

    extern bool pico_hx711_is_ready_unsafe(struct PicoHX711 *hx);
    extern bool pico_hx711_is_ready(struct PicoHX711 *hx);
    extern void pico_hx711_wait_ready_unsafe(struct PicoHX711 *hx);
    extern void pico_hx711_wait_ready(struct PicoHX711 *hx);
    extern bool pico_hx711_ready_timeout_unsafe(struct PicoHX711 *hx, uint32_t timeout_ms);
    extern bool pico_hx711_ready_timeout(struct PicoHX711 *hx, uint32_t timeout_ms);

    extern void pico_hx711_power_on_unsafe(struct PicoHX711 *hx);
    extern void pico_hx711_power_on(struct PicoHX711 *hx);
    extern void pico_hx711_power_off_unsafe(struct PicoHX711 *hx, bool wait_until_power_off);
    extern void pico_hx711_power_off(struct PicoHX711 *hx, bool wait_until_power_off);

    extern int32_t hx711_get_twos_comp(const uint32_t raw);

    extern bool pico_hx711_read_raw_single_unsafe(struct PicoHX711 *hx, int32_t *raw, uint32_t timeout_ms);
    extern bool pico_hx711_read_raw_single(struct PicoHX711 *hx, int32_t *raw, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* _PICO_HX711_H_ */