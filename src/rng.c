#include "rng.h"

//xorshift stops working if the state is 0, so a seed of 0 is swapped for a fixed nonzero const
void rng_seed(Rng *r, uint64_t seed)
{
    if(seed == 0){
        seed = 0x9E3779B97F4A7C15ULL; //backup seed , only if a seed of 0 is passed
                     //where 0x is written in hexadecimal, digits run 0-9 then A-F
                     //common for bit pattern writing
    }
    r->state = seed;
}

//three ^= lines scramble the bits (^ is XOR, << >> shifts bits left and right), new 
//state is saved back through the pointer, next call gives next number
double rng_uniform(Rng * r)
{
    uint64_t x = r->state;
    x ^= x << 13;           //bitwise XOR assignment operator to compute the bitwise
    x ^= x >>7;             //exclusive OR of a var and a value then stores result in that var
    x ^= x << 17;
    r->state = x;
    return(double)(x >> 11) / 9007199254740992.0;   //turns rand int into fraction between 0-1
}

//stretches the value of rng_uniform fractional value to fit b/w lo and hi
double rng_range(Rng *r, double lo, double hi)
{
    return lo + (hi - lo) * rng_uniform(r);
}

