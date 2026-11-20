#ifndef RNG_H
#define RNG_H

#include <stdint.h>

typedef struct{
    uint64_t state;
} Rng;

void rng_seed(Rng *r, uint64_t seed);
double rng_uniform(Rng *r);
double rng_range(Rng *r, double lo, double hi);

#endif