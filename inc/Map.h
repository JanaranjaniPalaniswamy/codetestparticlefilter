#ifndef _MAP_H_
#define _MAP_H_ _MAP_H_

#include <stdio.h>
#include <cstdint>
#include <cassert>
#include <iostream>
#include <bitset>

#include "ImageHandler.h"

/*
*The map is used by the sensor to perform raytracing operations and so on.
*A map inherits from the ImageHandler and computes binary image arrays based on an rgb input image.
*The map assumes that every pixel which is not (255,255,255) represents an obstacle.
*Thus, a binary obstacle arry is created internally. A '1' is an obstacle and a '0' means the cell
*is free.
*Moreover, a distance image transformation is computed internaly, which significantly speeds up raytracing
*operations.
*/

class Map : public ImageHandler
{
public:
    ~Map();
    /*
    *Load the map.
    *@param filename the name of the map file including its path.
    *@return true if loading the map was successful.
    */
    bool init(const char *filename);
    /*
    *Request the entry of a single cell of the binary obstacle map.
    *@param x the x-value of the cell
    *@param y the y-value of the cell
    *@param val the result is stored here
    *@return true if requesting the value was successful (i.e. x and y are within map boundaries)
    */
    bool value(uint32_t x, uint32_t y, uint8_t &val) const;
    /*
    *Request the value of a single cell of the distance map.
    *@param x the x-value of the cell
    *@param y the y-value of the cell
    *@param val the result is stored here
    *@return true if requesting the value was successful (i.e. x and y are within map boundaries)
    */
    bool value(uint32_t x, uint32_t y, double &val) const;

private:
    /*
    *When a map is loaded, artificial outer walls at the
    *boundaries are computed in order to keep distance values
    *small at the boundaries of the image. 
    */
    void drawOuterWalls();
    /*
    *The distance transformation is computed here.
    *The function is called by 'init'.
    */
    void computeDistMap();
    /*
    *Debug function to save the distance trafo as a bmp file.
    */
    void dbgSaveDistanceMap();

    uint8_t *_data{nullptr};
    double *_distData{nullptr};
};

#endif
