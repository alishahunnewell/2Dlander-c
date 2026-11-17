#include <stdio.h>
#include <math.h>
#include "state.h"
#include "params.h"
#include "dynamics.h"
#include "integrator.h"
#include "controller.h"
#include "metrics.h"

int main(void)
{
    //section 1: free fall, checks rk4 against the exact formula

    //setup section 1
    double z0 = 100.0;   //m
    State s = { 0.0, 100.0, 0.0, 1000.0, 0.0, 0.0};
    Params p = {-9.81, 0.01, 9.80665, 300.0, 20000.0, 5.0};
    Control u = {0.0, 0.0}; //no thrust
    double max_error = 0.0;

    //open output file 
    FILE *f = fopen("output/freefall.csv", "w");
    if (f == NULL){
        printf("could not open output file\n");
        return 1;
    }
    fprintf(f, "t,z,vz,z_exact\n");     

    //simulate until lander is on the ground
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


    //section 2: powered landing with three gain sets

    //setup shared by all three runs
    Params lp = {-9.81, 0.01, 9.80665, 300.0, 20000.0, 5.0};

    //gains in the order; kp_x, kd_x, kp_z, kd_z
    Gains gains[3] = {
        {0.5, 1.4, 1.0, 2.0},   // baseline
        {0.05, 0.0, 1.0, 2.0},   // weak horizontal spring no horiz damper
        {0.0, 0.0, 1.0, 2.0}    // no horiz control at all
    };

    //one output file per run
    const char *files[3] = {
        "output/landing_run0.csv",
        "output/landing_run1.csv",
        "output/landing_run2.csv"
    };


    //one pass of this loop = one full landing 
    for (int i = 0; i < 3; i++) {
        //starting state: 100 m up, 30 m off the pad, at rest
        State ls = {0.0, 100.0, 0.0, 1000.0, 30.0, 0.0};
        double m_start = ls.m;
        Control lu = {0.0, 0.0};

        //open this run's output file
        FILE *lf = fopen(files[i], "w");
        if (lf == NULL) {
            printf("could not open %s\n", files[i]);
            return 1;
        }
        fprintf(lf, "t,z,vz,Tz,x\n");

        //fly : controller -> clamp -> integrate -> log
        while (!has_touched_down(&ls) && ls.t < 60.0) {
            pd_control(&ls, &gains[i], &lp, &lu);
            clamp_thrust(&lu, &lp);
            ls = step_rk4(ls, lu, lp);
            fprintf(lf, "%f,%f,%f,%f,%f\n", ls.t, ls.z, ls.vz, lu.Tz, ls.x);
        }
        fclose(lf);

        // judge the landing and print the verdict
        Result r;
        evaluate_landing(&ls, m_start, &lp, &r);
        printf("run %d: ", i);
        print_result(&r);
    }
    
    return 0;
}
