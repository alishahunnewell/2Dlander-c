#include <stdio.h>
#include <math.h>
#include "state.h"
#include "params.h"
#include "dynamics.h"
#include "integrator.h"
#include "controller.h"
#include "metrics.h"
#include "sim.h"
#include "rng.h"

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
        //starting state, 100 m up, 30 m off pad and at rest
        State start = {0.0, 100.0, 0.0, 1000.0, 30.0, 0.0};

        //opening this run's output file
        FILE *lf = fopen(files[i], "w");
        if (lf == NULL) {
            printf("could not open %s\n", files[i]);
            return 1;
        }

        //fly landing and judge it
        Result r;
        run_landing(start, &gains[i], &lp, lf, &r);
        fclose(lf);

        printf("run %d: ", i);
        print_result(&r);
    }


    //section 3: monte carlo, random starts with baseline gains 
    //setup: seeded generator, run count, tallies 

    Rng rng;
    rng_seed(&rng, 42);


    int N = 1000;
    int n_landed = 0;
    int n_vz = 0;
    int n_vx = 0;
    int n_x = 0;
    int n_timeout = 0;

    Gains mc_gains = {0.5, 1.4, 1.0, 2.0};

    //worst failure tracker, declared here so it survives loo[]
    double worst_vz = 0.0;
    int worst_i = -1;       //means no failure has been found yet
    State worst_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //one pass = one random randing 
    for (int i = 0; i< N; i++){
        //random starting state, ranges from the paper table
        State start; 
        start.t = 0.0;
        start.z = rng_range(&rng, 80.0, 120.0);
        start.vz = rng_range(&rng, -10.0, 0.0);
        start.m = rng_range(&rng, 900.0, 1100.0);
        start.x = rng_range(&rng, -40.0, 40.0);
        start.vx = rng_range(&rng, -3.0, 3.0);

        //fly it with no csv then tally verdict w 
        Result r; 
        run_landing(start, &mc_gains, &lp, NULL, &r);

        if (r.landed){
            n_landed++;
        }   else if (r.reason == REASON_VZ) {
            n_vz++;
        }   else if (r.reason == REASON_VX) {
            n_vx++;
        }   else if (r.reason == REASON_X) {
            n_x++;
        }   else {
            n_timeout++;
        }

        //remember the failed run with the fastest touchdown
        if (!r.landed && fabs(r.vz_touch) > worst_vz) {
            worst_vz = fabs(r.vz_touch);
            worst_i = i;
            worst_start = start;
        }
    }

    //report section 3 monte carlo 
    printf("monte carlo: N=%d, seed=42\n", N);
    printf("  landed:           %d (%.1f%%)\n", n_landed, 100.0 * n_landed / N);
    printf("  vertical speed:   %d\n", n_vz);
    printf("  horizontal speed: %d\n", n_vx);
    printf("  missed pad:       %d\n", n_x);
    printf("  timeout:          %d\n", n_timeout);

    if (worst_i >= 0) {
        printf("worst failure: run %d, touchdown vz = %.2f m/s\n", worst_i, worst_vz);
        printf("  start: z=%.1f  vz=%.2f  m=%.1f  x=%.1f  vx=%.2f\n",
               worst_start.z, worst_start.vz, worst_start.m, worst_start.x, worst_start.vx);

        //fly the worst run again, this time saving a csv
        FILE *wf = fopen("output/worst.csv", "w");
        if (wf == NULL) {
            printf("could not open output/worst.csv\n");
            return 1;
        }
        Result wr;
        run_landing(worst_start, &mc_gains, &lp, wf, &wr);
        fclose(wf);

        printf("worst rerun: ");
        print_result(&wr);
    }

    return 0;
}
