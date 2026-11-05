#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "state.h"
#include "params.h"


StateDot derivatives(const State s, const Control u,const Params p);

#endif
