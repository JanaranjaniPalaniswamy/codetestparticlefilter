#ifndef _PARTICLE_H_
#define _PARTICLE_H_ _PARTICLE_H_

#include "Robot.h"

class Particle : public Robot
{
public:
    static constexpr double MEASUREMENT_VAR = 5.0 * 5.0;

    /*
    *Compute the weight for each particle here given
    *the measurement from the sensor.
    *@param measurement
    */
    void computeWeight(const double *measurement);
    /*
    *Get the particle weight.
    */
    double weight();
    /*
    *Set the particle weight from outside.
    *@param weight
    */
    void setWeigth(double weight);
    /*
    *Define a pose for the particle. Note, that
    *this is not a delta motion but an absolute pose.
    *@param x the x position
    *@param y the y position 
    *@param yaw the heading of the particle [rad]
    */
    void setPose(double x, double y, double yaw);

    /*
    *Definition of an assignment operator.
    */
    Particle &operator=(Particle &p);

private:
    double _weight;
};

#endif