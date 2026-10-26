#include <stdio.h>
#include <math.h>

//explicit euler method applied to free fall, scope: vertical only, const g, no thrust no change in mass

int main(void)
{
    //starting values t, z, vz, g, dt
    double g = -9.81;
    double t = 0.0;
    double z0 = 100.0;   //m
    double z = 100.0;
    double vz = 0.0;
    double dt = 0.01;
    printf("%f %f %f \n", t, z, vz);

    double max_error = 0.0;

    // loop while lander is above the ground
    while(z > 0){
        z = z + vz * dt;
        vz = vz + g*dt;
        t = t + dt;

        double height_exact = z0 + 0.5 * g * t * t;
        double computed_gap = fabs(z - height_exact);   //where fabs() returns abs value of float

        if(computed_gap > max_error){
            max_error = computed_gap;
        }

        // printf("%f %f %f %f \n", t, z, vz, height_exact);
        //prints row of numbers
    }
    printf("max error: %f\n", max_error);
    return 0;

}