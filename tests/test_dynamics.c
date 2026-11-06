#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "dynamics.h"
#include "state.h"
#include "params.h"
#include "integrator.h"


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

    //burn test 
    State s = {0.0, 0.0, 0.0, 1000.0};
    Params q = {0.0, 0.01, 9.80665, 300.0, 20000.0};
    Control burn = {0.0, 20000.0};

    double expected = 0.0; //variable decleration in outer scope so expected is alive outsid loop
    
    while(s.m > 800.0){
        double m_prev = s.m;    //snapshot of mass just before step
        s = step_rk4(s, burn, q);
        assert(s.m <= m_prev);  //mass should be decreasing
    }

    expected = q.Isp * q.g0 * log(1000.0 / s.m);
    assert (fabs(s.vz - expected) / expected < 0.001);
    printf("s.vz: %f, expected: %f\n", s.vz, expected);
    printf("test_dynamics.c passed!\n");

    return 0;
}