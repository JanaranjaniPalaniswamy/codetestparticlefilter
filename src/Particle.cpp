#include "Particle.h"
#include <cmath>

void Particle::computeWeight(const double *measurement)
{
     ////////////////////////////////////////////////////////////////////////////
    //////////YOUR CODE GOES HERE!!!////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
}

double Particle::weight()
{
    return _weight;
}
void Particle::setWeigth(double weight)
{
    _weight = weight;
}

void Particle::setPose(double x, double y, double yaw)
{
    _x = x;
    _y = y;
    _yaw = yaw;

    if (_sensor != nullptr)
    {
        _sensor->updatePose(_x, _y, _yaw);
    }
}

Particle &Particle::operator=(Particle &p)
{
    _x = p.x();
    _y = p.y();
    _yaw = p.yaw();
    _weight = p.weight();
    return *this;
}