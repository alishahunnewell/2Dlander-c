#ifndef METRICS_H
#define METRICS_H

#include <stdbool.h>
#include "state.h"
#include "params.h"

#define REASON_NONE 0
#define REASON_VZ   1
#define REASON_VX   2
#define REASON_X    3
#define REASON_TIMEOUT 4

typedef struct {
    double vz_touch;
    double vx_touch;
    double x_error;
    double fuel_used;
    double flight_time;
    bool landed;
    int reason;
} Result;

bool has_touched_down(const State *s);
void evaluate_landing(const State *final, double m_initial, const Params *p, Result *out);
void print_result(const Result *r);

#endif
