#ifndef STATE_H
#define STATE_H


typedef struct{
    double t;
    double z;
    double vz;
    double m;
} State;

typedef struct {
    double dz;  //how fast z is changing    
    double dvz; //how fast vz is changing
    double dm;
} StateDot;

#endif