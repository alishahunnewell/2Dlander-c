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

State step_euler(State s, Params p)
{
    StateDot d =  derivatives(s, p);
    s.z = s.z + d.dz * p.dt;
    s.vz = s.vz + d.dvz*p.dt; 
    s.t = s.t + p.dt;
    return s;

}

State step_rk4(State s, Params p)
{
    StateDot k1 = derivatives(s, p);

    State s2 = s;
    s2.z = s.z + 0.5 * p.dt *k1.dz;
    s2.vz = s.vz + 0.5 * p.dt * k1.dvz;
    StateDot k2 = derivatives(s2, p);

    State s3 = s;
    s3.z = s.z + 0.5 * p.dt *k2.dz;
    s3.vz = s.vz + 0.5 * p.dt * k2.dvz;
    StateDot k3 = derivatives(s3, p);

    State s4 = s;  //full step so remove the 0.5 
    s4.z = s.z  +  p.dt *k3.dz;
    s4.vz = s.vz + p.dt * k3.dvz;
    StateDot k4 = derivatives(s4, p);

    //combine 
    s.z = s.z + p.dt/6 * (k1.dz + 2*k2.dz + 2*k3.dz + k4.dz);
    s.vz = s.vz + p.dt/6 * (k1.dvz + 2*k2.dvz + 2*k3.dvz + k4.dvz);
    s.t = s.t + p.dt;

    return s;

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