#include "dynamics.h"

StateDot derivatives(const State s, const Params p)
{
    StateDot out;
    out.dz  = s.vz;
    out. dvz = p.g;
    out.dm = 0.0; //no change in mass for now
    return out;
}