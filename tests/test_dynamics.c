#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "dynamics.h"
#include "state.h"
#include "params.h"


int main(void)
{
    Control u = {30000.0 , 40000.0};
    Params p = {-9.81, 0.01, 9.80665, 300.0, 20000.0};
    clamp_thrust(&u, &p);
    assert((sqrt(u.Tx * u.Tx + u.Tz * u.Tz)) <= p.Tmax);
    
    u = (Control){0.0, -500.0};
    clamp_thrust(&u, &p);
    assert(u.Tz >= 0.0);

    u = (Control){0.0, 10000.0};
    StateDot d = derivatives((State){0.0, 100.0, 0.0, 1000.0}, u, p);
    assert(d.dm < 0.0);

    printf("test_dynamics.c passed!\n");

    return 0;
}