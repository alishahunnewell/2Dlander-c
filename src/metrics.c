#include "metrics.h"
#include <math.h>
#include <stdio.h>

bool has_touched_down(const State *s)
{
    return s->z <= 0.0;
}

void evaluate_landing(const State *final, double m_initial, const Params *p, Result *out)
{
    out->vz_touch = final->vz;
    out->vx_touch = final->vx;
    out->x_error = fabs(final->x);
    out->fuel_used = m_initial - final->m;
    out->flight_time = final->t;

    if (final->z > 0.0) {
        out->landed = false;
        out->reason = REASON_TIMEOUT;
    } else if (fabs(final->vz) >= 2.0) {
        out->landed = false;
        out->reason = REASON_VZ;
    } else if (fabs(final->vx) >= 1.0) {
        out->landed = false;
        out->reason = REASON_VX;
    } else if (out->x_error > p->pad_radius) {
        out->landed = false;
        out->reason = REASON_X;
    } else {
        out->landed = true;
        out->reason = REASON_NONE;
    }
}

void print_result(const Result *r)
{
    if (r->landed) {
        printf("LANDED: vz=%.2f m/s, vx=%.2f m/s, %.2f m from pad, fuel %.1f kg, %.2f s\n",
               r->vz_touch, r->vx_touch, r->x_error, r->fuel_used, r->flight_time);
    } else if (r->reason == REASON_VZ) {
        printf("CRASHED: vertical speed %.2f m/s (limit 2.0)\n", fabs(r->vz_touch));
    } else if (r->reason == REASON_VX) {
        printf("CRASHED: horizontal speed %.2f m/s (limit 1.0)\n", fabs(r->vx_touch));
    } else if (r->reason == REASON_X) {
        printf("CRASHED: landed %.2f m from pad center\n", r->x_error);
    } else {
        printf("TIMEOUT: never reached the ground\n");
    }
}