#include <stdio.h>
#include <math.h>
#include "state.h"
#include "params.h"
#include "dynamics.h"
#include "integrator.h"
#include "controller.h"

//explicit euler method applied to free fall, scope: vertical only, const g, no thrust no change in mass
int main(void)
{
    //section 1:free fall code
    double z0 = 100.0;   //m
    State s = { 0.0, 100.0, 0.0, 1000.0};
    Params p = {-9.81, 0.01, 9.80665, 300.0, 20000.0};
    Control u = {0.0, 0.0}; //no thrust
    double max_error = 0.0;

    FILE *f = fopen("output/freefall.csv", "w");
    if (f == NULL){
        printf("could not open output file\n");
        return 1;
    }
    fprintf(f, "t,z,vz,z_exact\n");     

    while(s.z > 0){
        s = step_rk4(s, u, p);

        double height_exact = z0 + 0.5 * p.g * s.t * s.t;
        double computed_gap = fabs(s.z - height_exact);   //where fabs() returns abs value of float
        fprintf(f, "%f,%f,%f,%f\n", s.t, s.z, s.vz, height_exact);

        if(computed_gap > max_error){
            max_error = computed_gap;
        }

    }
    
    fclose(f);
    printf("max error: %e\n", max_error);

    Params lp = {-9.81, 0.01, 9.80665, 300.0, 20000.0};

    Gains gains[3] = {
    {0.0, 0.0, 1.0, 2.0},   // baseline
    {0.0, 0.0, 1.0, 0.0},   // Kd = 0 (no damper)
    {0.0, 0.0, 2.0, 2.0}    // Kp doubled
    };
    const char *files[3] = {
    "output/landing_base.csv",
    "output/landing_kd0.csv",
    "output/landing_kp2.csv"
    };

    for (int i = 0; i < 3; i++) {
    State ls = {0.0, 100.0, 0.0, 1000.0};
    Control lu = {0.0, 0.0};

    FILE *lf = fopen(files[i], "w");
    if (lf == NULL) {
        printf("could not open %s\n", files[i]);
        return 1;
    }
    fprintf(lf, "t,z,vz,Tz\n");

    while (ls.z > 0.0 && ls.t < 60.0) {
        pd_control(&ls, &gains[i], &lp, &lu);
        clamp_thrust(&lu, &lp);
        ls = step_rk4(ls, lu, lp);
        fprintf(lf, "%f,%f,%f,%f\n", ls.t, ls.z, ls.vz, lu.Tz);
    }

    fclose(lf);
    printf("run %d: t=%f  vz=%f\n", i, ls.t, ls.vz);
    }
    
    
    return 0;
}
