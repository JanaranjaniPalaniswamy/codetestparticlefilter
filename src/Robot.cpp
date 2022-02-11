#include "Robot.h"

void Robot::init(std::shared_ptr<Sensor> sensor, double x, double y, double yawRad)
{
    _sensor = sensor;
    _x = x;
    _y = y;
    _yaw = yawRad;
    //Assume the transformation Robot_T_Sensor to be the unity matrix
    _sensor->updatePose(_x, _y, _yaw);
}

void Robot::move(double dx, double dy, double dYaw)
{
    double c_a = cos(_yaw);
    double s_a = sin(_yaw);

    _x += dx * c_a - dy * s_a;
    _y += dx * s_a + dy * c_a;
    _yaw += dYaw;
    _yaw = fmod(_yaw, M_PI * 2);
    if (_sensor != nullptr)
    {
        _sensor->updatePose(_x, _y, _yaw);
    }
}

uint32_t Robot::totalNumberOfMeasurements()
{
    if (_sensor != nullptr)
    {
        return _sensor->NUM_SENSOR_MEASUREMENTS;
    }
    else
    {
        return 0;
    }
}

const double *Robot::perceiveEnvironment()
{
    if (_sensor != nullptr)
    {
        return _sensor->doMeasurement();
    }
    else
    {
        return nullptr;
    }
}

double Robot::x() const
{
    return _x;
}

double Robot::y() const
{
    return _y;
}

double Robot::yaw() const
{
    return _yaw;
}