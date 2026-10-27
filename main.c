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

//we want to pull s.vz rate of z, and p.g rate of vz so rk4 can ask;
    //what are the rates at this state? four times 
    //use another struct for this
typedef struct {
    double dz;  //how fast z is changing    
    double dvz; //how fast vz is changing
} StateDot;

//forward decleration of statedot dunction
StateDot derivatives(const State s, const Params p)
{
    StateDot out;
    out.dz  = s.vz;
    out. dvz = p.g;
    return out;
}

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
        StateDot d = derivatives(s, p);
        s.z = s.z + d.dz * p.dt;
        s.vz = s.vz + d.dvz*p.dt;         //z and vz are loose variables
        //we can bundle loose vars into a struct with a name change
        //bundling loose vars into struct to prep for rk4, and in C the clean way to 
        //return a bundle of numbers is with a struct
        s.t = s.t + p.dt;

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