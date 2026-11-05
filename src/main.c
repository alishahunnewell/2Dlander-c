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
    State s = { 0.0, 100.0, 0.0, 1000.0};
    Params p = {-9.81, 0.01, 9.80665, 300.0, 20000.0};
    double max_error = 0.0;

    FILE *f = fopen("output/freefall.csv", "w");
    if (f == NULL){
        printf("could not open output file\n");
        return 1;
    }
    fprintf(f, "t,z,vz,z_exact\n");     

    while(s.z > 0){
        s = step_rk4(s, p);

        double height_exact = z0 + 0.5 * p.g * s.t * s.t;
        double computed_gap = fabs(s.z - height_exact);   //where fabs() returns abs value of float
        fprintf(f, "%f,%f,%f,%f\n", s.t, s.z, s.vz, height_exact);

        if(computed_gap > max_error){
            max_error = computed_gap;
        }

    }
    
    fclose(f);
    printf("max error: %e\n", max_error);
    return 0;
}
