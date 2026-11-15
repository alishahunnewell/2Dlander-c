#ifndef STATE_H
#define STATE_H


typedef struct{
    double t;
    double z;
    double vz;
    double m;
    double x;
    double vx;
} State;

typedef struct {
    double dz;  //how fast z is changing    
    double dvz; //how fast vz is changing
    double dm;
    double dx;
    double dvx;
} StateDot;

typedef struct{
    double Tx; //horizontal thrust 
    double Tz; //vertical thrust 
} Control;

#endif