#include "PicoHX711.h"

const uint16_t HX711_SETTLING_TIMES_MS[] = {
    400, // 10 SPS
    50   // 80 SPS
};


void pico_hx711_begin(struct PicoHX711 *hx, uint8_t pin_clock, uint8_t pin_data,
                      enum PicoHX711Gain gain, enum PicoHX711Rate rate)
{
    critical_section_init(&hx->cs);
    // if (!mutex_is_initialized(&hx->mux))
    mutex_init(&hx->mux);

    mutex_enter_blocking(&hx->mux);

    hx->pin_clock = pin_clock;
    hx->pin_data = pin_data;
    hx->gain = gain;
    hx->rate = rate;

    gpio_init(hx->pin_clock);
    gpio_set_dir(hx->pin_clock, GPIO_OUT);

    gpio_init(hx->pin_data);
    gpio_set_dir(hx->pin_data, GPIO_IN);

    mutex_exit(&hx->mux);
}

bool pico_hx711_is_ready_unsafe(struct PicoHX711 *hx)
{
    return gpio_get(hx->pin_data) == false;
}

bool pico_hx711_is_ready(struct PicoHX711 *hx)
{
    mutex_enter_blocking(&hx->mux);
    bool res = pico_hx711_is_ready_unsafe(hx);
    mutex_exit(&hx->mux);
    return res;
}

void pico_hx711_wait_ready_unsafe(struct PicoHX711 *hx)
{
    while (!pico_hx711_is_ready_unsafe(hx))
        sleep_us(HX711_IS_READY_POLLING_DELAY_US);
}

void pico_hx711_wait_ready(struct PicoHX711 *hx)
{
    mutex_enter_blocking(&hx->mux);
    pico_hx711_wait_ready_unsafe(hx);
    mutex_exit(&hx->mux);
}

bool pico_hx711_ready_timeout_unsafe(struct PicoHX711 *hx, uint32_t timeout_ms)
{
    absolute_time_t end_time = make_timeout_time_ms(timeout_ms);
    while (!time_reached(end_time))
    {
        if (pico_hx711_is_ready_unsafe(hx)) return true;
        sleep_us(HX711_IS_READY_POLLING_DELAY_US);
    }
    return false;
}

bool pico_hx711_ready_timeout(struct PicoHX711 *hx, uint32_t timeout_ms)
{
    mutex_enter_blocking(&hx->mux);
    bool res = pico_hx711_ready_timeout_unsafe(hx, timeout_ms);
    mutex_exit(&hx->mux);
    return res;
}

void pico_hx711_power_on_unsafe(struct PicoHX711 *hx)
{
    gpio_put(hx->pin_clock, false);
}

void pico_hx711_power_on(struct PicoHX711 *hx)
{
    mutex_enter_blocking(&hx->mux);
    pico_hx711_power_on_unsafe(hx);
    mutex_exit(&hx->mux);
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
    mutex_enter_blocking(&hx->mux);
    pico_hx711_power_off_unsafe(hx, wait_until_power_off);
    mutex_exit(&hx->mux);
}

static inline void pico_hx711_pulse(struct PicoHX711 *hx)
{
    gpio_put(hx->pin_clock, true);
    sleep_us(HX711_PULSE_DELAY_US);
    gpio_put(hx->pin_clock, false);
    sleep_us(HX711_PULSE_DELAY_US);
}

#define HX711_MIN_VALUE INT32_C(-0x800000) //−8,388,608
#define HX711_MAX_VALUE INT32_C(0x7fffff) //8,388,607
int32_t hx711_get_twos_comp(const uint32_t raw) {
    return
        (int32_t)(-(raw & +HX711_MIN_VALUE)) + 
        (int32_t)(raw & HX711_MAX_VALUE);
}

#define HX711_READ_BITS UINT8_C(24)
bool pico_hx711_read_raw_single_unsafe(struct PicoHX711 *hx, int32_t *raw, uint32_t timeout_ms)
{
    uint32_t data;

    pico_hx711_power_on_unsafe(hx);

    if (timeout_ms > 0)
    {
        if (!pico_hx711_ready_timeout_unsafe(hx, timeout_ms))
            return false;
    }
    else
        pico_hx711_wait_ready_unsafe(hx);

    // read
    critical_section_enter_blocking(&hx->cs);
    for (uint8_t i = 0; i < HX711_READ_BITS; ++i)
    {
        pico_hx711_pulse(hx);
        data |= gpio_get(hx->pin_data) << (HX711_READ_BITS - i);
    }

    // set gain
    for (uint8_t i = 0; i < hx->gain; ++i)
        pico_hx711_pulse(hx);
    critical_section_exit(&hx->cs);

    // to two's compliment
    *raw = hx711_get_twos_comp(data);
    return true;
}

bool pico_hx711_read_raw_single(struct PicoHX711 *hx, int32_t *raw, uint32_t timeout_ms)
{
    mutex_enter_blocking(&hx->mux);
    bool res = pico_hx711_read_raw_single_unsafe(hx, raw, timeout_ms);
    mutex_exit(&hx->mux);
    return res;
}
