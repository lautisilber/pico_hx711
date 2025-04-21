//
//  Welfords.h
//  stoma_sense_algorithm
//
//  Created by Lautaro Silbergleit on 12/04/2025.
//

#ifndef _WELFORDS_H_
#define _WELFORDS_H_

#include "pico/stdlib.h"

// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance

#ifdef __cplusplus
extern "C"
{
#endif

struct WelfordsAggregate {
    uint32_t count;
    float mean, M2;
};

extern void welfords_update(struct WelfordsAggregate *agg, float new_value);
extern bool welfords_finalize(struct WelfordsAggregate *agg, float *mean, float *stdev);

#ifdef __cplusplus
}
#endif

#endif /* _WELFORDS_H_ */