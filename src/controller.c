#include "controller.h"

void pd_control(const State *s, const Gains *k, const Params *p, Control *out)
{
    double z_target = -1.0; //pd controller at 0 may never cross and loop may not end

    //desired acceleration 
    double a_cmd = k->kp_z * (z_target - s->z) + k->kd_z * (0.0 - s->vz);

    //gravity compensation
    a_cmd = a_cmd - p->g;
    out->Tz = s->m * a_cmd;
    out->Tx = 0.0;
    
}