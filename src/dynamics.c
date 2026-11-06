#include "dynamics.h"
#include <math.h>
StateDot derivatives(const State s, const Control u,const Params p)
{
    StateDot out;
    out.dz  = s.vz;
    out. dvz = u.Tz / s.m + p.g; //updated body
    out.dm = -sqrt(u.Tx * u.Tx + u.Tz * u.Tz) / (p.Isp * p.g0);
    return out;
}
//limits control so that abs(T) is less than equal to Tmax and Tz is less than or equal to 0
void clamp_thrust(Control *u, const Params *p)
{
    if (u->Tz < 0){
        u->Tz = 0;
    }
    
    double magnitude = sqrt(u->Tx * u->Tx + u->Tz * u->Tz);

    if (magnitude > p->Tmax) {
        double scale = p->Tmax / magnitude;
        u->Tx *= scale;
        u->Tz *= scale;
    }
    
}