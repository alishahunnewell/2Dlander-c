#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "state.h"
#include "params.h"

typedef struct {
    double kp_x;
    double kd_x;
    double kp_z;
    double kd_z;
} Gains;

void pd_control(const State *s, const Gains *k, const Params *p, Control *out);

#endif