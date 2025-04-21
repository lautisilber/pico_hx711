#ifndef _HX711_UTILS_H_
#define _HX711_UTILS_H_


#define HX711_MUTEX_BLOCK(mut, ...) \
    do                              \
    {                               \
        mutex_enter_blocking(&mut); \
        __VA_ARGS__                 \
        mutex_exit(&mut);           \
    } while (0)

#define HX711_INTERRUPTS_OFF_BLOCK(...)                                                                       \
    do                                                                                                        \
    {                                                                                                         \
        const uint32_t interrupt_status_59f409f6_bd91_4de5_99b7_0abc6b173fe8 = save_and_disable_interrupts(); \
        __VA_ARGS__                                                                                           \
        restore_interrupts(interrupt_status_59f409f6_bd91_4de5_99b7_0abc6b173fe8);                            \
    } while (0)

#endif /* _HX711_UTILS_H_ */