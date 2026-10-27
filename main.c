#include <stdio.h>
#include <math.h>

//state struct
typedef struct{
    double t;
    double z;
    double vz;
} State;

//params struct 
typedef struct {
    double g;
    double dt;
} Params;

//explicit euler method applied to free fall, scope: vertical only, const g, no thrust no change in mass
int main(void)
{
    //starting values t, z, vz, g, dt
    // double g = -9.81;
    // double t = 0.0;
    double z0 = 100.0;   //m
    // double z = 100.0;
    // double vz = 0.0;
    // double dt = 0.01;
    //become two lines that create the bundles filled in order
    State s = { 0.0, 100.0, 0.0};
    Params p = {-9.81, 0.01};
    printf("%f %f %f \n", s.t, s.z, s.vz);

    double max_error = 0.0;

    // loop while lander is above the ground
    while(s.z > 0){
        s.z = s.z + s.vz * p.dt;
        s.vz = s.vz + p.g*p.dt;         //z and vz are loose variables

        //we can bundle loose vars into a struct with a name change
        //bundling loose vars into struct to prep for rk4, and in C the clean way to 
        //return a bundle of numbers is with a struct
        s.t = s.t + p.dt;

        //bundled
        

        double height_exact = z0 + 0.5 * p.g * s.t * s.t;
        double computed_gap = fabs(s.z - height_exact);   //where fabs() returns abs value of float

        if(computed_gap > max_error){
            max_error = computed_gap;
        }

        // printf("%f %f %f %f \n", t, z, vz, height_exact);
        //prints row of numbers
    }
    printf("max error: %f\n", max_error);
    return 0;

}