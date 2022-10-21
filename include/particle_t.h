/*******************************************************************************************
 * File:        particle_t.h
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     05/19/2022
 * 
 * Description: Struct for particles
*******************************************************************************************/

#ifndef PARTICLE_STRUCT_H
#define PARTICLE_STRUCT_H

typedef struct particle
{
    int particle_id;
    double x;
    double y;
    double theta;
    double weight;
} particle_t;

#endif /* PARTICLE_STRUCT_H */