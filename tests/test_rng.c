#include <assert.h>
#include <stdio.h>
#include "rng.h"

int main(void){
    Rng a;
    Rng b;
    rng_seed(&a, 42);
    rng_seed(&b, 42);

    for (int i = 0; i < 1000; i++){
        double x = rng_uniform(&a);
        double y = rng_uniform(&b);
        assert(x == y);
        assert(x >= 0.0 && x < 1.0);
    }

    Rng c;
    rng_seed(&c, 42);
    for (int i = 0; i < 1000; i++){
        double v = rng_range(&c, 80.0, 120.0);
        assert(v >= 80.0 && v < 120.0);
    }
    printf("test_rng passed!\n");
    return 0;
}
