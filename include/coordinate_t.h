/*******************************************************************************************
 * File:        coordinate_t.h
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     05/19/2022
 * 
 * Description: struct for metric coordinate after parsing longtitude and latitude.
*******************************************************************************************/

#ifndef COORDINATE_STRUCT_H
#define COORDINATE_STRUCT_H
#define DIM 4
typedef struct coordinate
{
    double val_[DIM];
} coordinate_t;

#endif /* COORDINATE_STRUCT_H */