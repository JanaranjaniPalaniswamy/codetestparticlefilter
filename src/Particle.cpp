#include "Particle.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include "ParticleFilter.h"

#define z_hit     0.7
#define z_short   0.12
#define z_max     0.05
#define z_rand    0.19
#define lambda_short 0.005

void Particle::computeWeight(const double *measurement, uint32_t length)
{
     ////////////////////////////////////////////////////////////////////////////
    //////////YOUR CODE GOES HERE!!!////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    double q = 1;
    double p, p_h, p_s, p_m, p_r;
    const double *z_k_star;

    z_k_star = _sensor->doMeasurement();

    int arrSize = sizeof(z_k_star) / sizeof(z_k_star[0]);

    for (int k = 1; k < arrSize; k++)
    {
        
        p_h = p_hit(measurement[k], z_k_star[k]);
        p_s = p_short(measurement[k], z_k_star[k]);
        p_m = p_max(measurement[k]);
        p_r = p_rand(measurement[k]);

        p = z_hit * p_h + z_short * p_s + z_max * p_m + z_rand * p_r;

        q = q * p;

        q += (-log(1 - p));
    }

    _weight = q;
    
}

double Particle::p_hit(const double& measurement, const double& predict_measurement)
{
    double eta = 1 / sqrt(2 * M_PI * Particle::MEASUREMENT_VAR);

    double diff = measurement - predict_measurement;

    double p_hit = eta * exp(-0.5 * diff * diff / Particle::MEASUREMENT_VAR);

    return (measurement >= 0 && measurement <= Sensor::NUM_SENSOR_MEASUREMENTS) ? p_hit : 0.0;
}

double Particle::p_short(const double& measurement, const double& predict_measurement)
{
    double eta = 1 / (1- exp(-lambda_short * predict_measurement));
        
    double p_short = eta * lambda_short * exp(lambda_short * measurement);

    return (measurement >= 0 && measurement <= predict_measurement) ? p_short : 0.0;
}

double Particle::p_max(const double& measurement)
{
    return (measurement >= Sensor::NUM_SENSOR_MEASUREMENTS) ? 1.0 : 0.0;
}

double Particle::p_rand(const double& measurement)
{
    return (measurement >= 0 && measurement < Sensor::NUM_SENSOR_MEASUREMENTS) ? 1 / Sensor::NUM_SENSOR_MEASUREMENTS : 0.0;
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