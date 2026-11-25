#ifndef SIM_H
#define SIM_H

#include <stdio.h>
#include "state.h"
#include "params.h"
#include "controller.h"
#include "metrics.h"

void run_landing(State start, const Gains *k, const Params *p, FILE *log, Result *out);

#endif
