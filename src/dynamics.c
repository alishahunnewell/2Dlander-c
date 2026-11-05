#include "dynamics.h"

StateDot derivatives(const State s, const Control u,const Params p)
{
    StateDot out;
    out.dz  = s.vz;
    out. dvz = u.Tz / s.m + p.g; //updated body
    out.dm = -u.Tz / (p.Isp * p.g0);  //no change in mass for now
    return out;
}