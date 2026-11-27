#include "sim.h"
#include "dynamics.h"
#include "integrator.h"

void run_landing(State start, const Gains *k, const Params *p, FILE *log, Result *out)
{
    State s = start;
    double m_start = s.m;
    Control u = {0.0, 0.0};

    if (log != NULL){
        fprintf(log, "t, z, vz, Tz, x, Tx\n");
    }

    while (!has_touched_down(&s) && s.t < 60.0) {
        pd_control(&s, k, p, &u);
        clamp_thrust(&u, p);
        s = step_rk4(s, u, *p);
        if (log != NULL){
            fprintf(log, "%f,%f,%f,%f,%f,%f\n", s.t, s.z, s.vz, u.Tz, s.x, u.Tx);
        }
    }

    evaluate_landing(&s, m_start, p, out);
}