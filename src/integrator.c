#include "integrator.h"
#include "dynamics.h"


State step_euler(State s, Control u, Params p)
{
    StateDot d =  derivatives(s, u, p);
    s.z = s.z + d.dz * p.dt;
    s.vz = s.vz + d.dvz*p.dt;
    s.m = s.m + d.dm * p.dt;
    s.t = s.t + p.dt;
    return s;

}

State step_rk4(State s, Control u, Params p)
{
    StateDot k1 = derivatives(s, u, p);

    State s2 = s;
    s2.z = s.z + 0.5 * p.dt *k1.dz;
    s2.vz = s.vz + 0.5 * p.dt * k1.dvz;
    s2.m = s.m + 0.5 * p.dt * k1.dm;
    StateDot k2 = derivatives(s2, u, p);

    State s3 = s;
    s3.z = s.z + 0.5 * p.dt *k2.dz;
    s3.vz = s.vz + 0.5 * p.dt * k2.dvz;
    s3.m = s.m + 0.5 * p.dt * k2.dm;
    StateDot k3 = derivatives(s3, u, p);

    State s4 = s;  //full step so remove the 0.5 
    s4.z = s.z  +  p.dt *k3.dz;
    s4.vz = s.vz + p.dt * k3.dvz;
    s4.m = s.m + p.dt * k3.dm;
    StateDot k4 = derivatives(s4, u, p);

    //combine 
    s.z = s.z + p.dt/6 * (k1.dz + 2*k2.dz + 2*k3.dz + k4.dz);
    s.vz = s.vz + p.dt/6 * (k1.dvz + 2*k2.dvz + 2*k3.dvz + k4.dvz);
    s.m = s.m + p.dt/6 * (k1.dm + 2*k2.dm + 2*k3.dm + k4.dm);
    s.t = s.t + p.dt;

    return s;

}