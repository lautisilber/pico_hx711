//
//  Welfords.cpp
//  stoma_sense_algorithm
//
//  Created by Lautaro Silbergleit on 12/04/2025.
//

#include "welfords.h"
#include <math.h>

void welfords_update(struct WelfordsAggregate *agg, float new_value)
{
    float delta, delta2;
    agg->count += 1;
    delta = new_value - agg->mean;
    agg->mean += delta / agg->count;
    delta2 = new_value - agg->mean;
    agg->M2 += delta * delta2;
}

bool welfords_finalize(struct WelfordsAggregate *agg, float *mean, float *stdev)
{
    if (agg->count < 2) return false;
    (*mean) = agg->mean;
    // float variance = agg->M2 / agg->count;
    float sample_variance = agg->M2 / (agg->count - 1);
    (*stdev) = sqrt(sample_variance);
    return true;
}