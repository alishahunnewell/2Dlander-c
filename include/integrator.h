#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "state.h"
#include "params.h"

State step_euler(State s, Params p);
State step_rk4(State s, Params p);


#endif