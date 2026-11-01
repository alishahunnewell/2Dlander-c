#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "state.h"
#include "params.h"


StateDot derivatives(const State s, const Params p);

#endif
