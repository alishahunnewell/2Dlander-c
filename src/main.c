#include <stdio.h>
#include <math.h>
#include "state.h"
#include "params.h"
#include "dynamics.h"
#include "integrator.h"

//explicit euler method applied to free fall, scope: vertical only, const g, no thrust no change in mass
int main(void)
{
    double z0 = 100.0;   //m
    State s = { 0.0, 100.0, 0.0};
    Params p = {-9.81, 0.01};
    printf("%f %f %f \n", s.t, s.z, s.vz);

    double max_error = 0.0;

    // loop while lander is above the ground
    while(s.z > 0){
        s = step_rk4(s, p);

        double height_exact = z0 + 0.5 * p.g * s.t * s.t;
        double computed_gap = fabs(s.z - height_exact);   //where fabs() returns abs value of float

        if(computed_gap > max_error){
            max_error = computed_gap;
        }

        // printf("%f %f %f %f \n", t, z, vz, height_exact);
        //prints row of numbers
    }
    printf("max error: %e\n", max_error);
    return 0;
}
//note euler error was 0.222 and rk4 is effectively 0