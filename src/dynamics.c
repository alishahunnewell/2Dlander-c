#include "dynamics.h"

StateDot derivatives(const State s, const Params p)
{
    StateDot out;
    out.dz  = s.vz;
    out. dvz = p.g;
    return out;
}