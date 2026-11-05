#include <stdio.h>
#include <math.h>
#include "state.h"
#include "params.h"
#include "dynamics.h"
#include "integrator.h"
#include <assert.h>

int main(void)
{
    double z0 = 100.0;   //m
    State s = { 0.0, 100.0, 0.0, 1000.0};
    Params p = {-9.81, 0.01, 9.80665, 300.0, 20000.0};
    Control u = {0.0, 0.0}; //no thrust
    double max_error = 0.0;

    // loop while lander is above the ground
    while(s.z > 0){
        s = step_rk4(s, u, p);

        double height_exact = z0 + 0.5 * p.g * s.t * s.t;
        double computed_gap = fabs(s.z - height_exact);   //where fabs() returns abs value of float

        if(computed_gap > max_error){
            max_error = computed_gap;
        }

        // printf("%f %f %f %f \n", t, z, vz, height_exact);
        //prints row of numbers
    }
    assert(max_error < 1e-6);
    printf("The test passed!\n");
    return 0;
}