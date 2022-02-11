

#include "Sensor.h"

Sensor::Sensor(std::shared_ptr<Map> mapPtr)
{
    _mapPtr = mapPtr;
}

void Sensor::updatePose(double x, double y, double yawRad)
{
    _sensorPosX = x;
    _sensorPosY = y;
    _sensorYaw = yawRad;
}

const double *Sensor::doMeasurement()
{
    double th = _sensorYaw - M_PI_2; //start from sensor orientation - 90°

    for (int i = 0; i < NUM_SENSOR_MEASUREMENTS; ++i)
    {
        //doRaytracing(_sensorPosX, _sensorPosY, th, _measurement[i]);
        doRaytracingFast(_sensorPosX, _sensorPosY, th, _measurement[i]);
        th += ANGULAR_RESOLUTION;
    }

    return _measurement;
}

bool Sensor::doRaytracing(double x, double y, double yawRad, double &res)
{
    double dx, dy;

    double len = 1.0f; //unit vector length

    bool wallFound = false;

    while (!wallFound)
    {
        double current_x = x + len * cos(yawRad);
        double current_y = y + len * sin(yawRad);
        uint32_t mx = (uint32_t)std::ceil(current_x);
        uint32_t my = (uint32_t)std::ceil(current_y);

        if (mx >= _mapPtr->width() || my >= _mapPtr->height())
        {
            dx = x - current_x;
            dy = y - current_y;
            res = sqrt(dx * dx + dy * dy);
            return false;
        }

        uint8_t val{0};
        if (!_mapPtr->value(mx, my, val))
        {
            std::cout << "Error getting map value" << std::endl;
        }

        if (val)
        {
            dx = x - current_x;
            dy = y - current_y;

            res = sqrt(dx * dx + dy * dy);
            wallFound = true;
        }
        len += MEASUREMENT_STEP;
    }

    return true;
}

bool Sensor::doRaytracingFast(double x, double y, double yawRad, double &res)
{

    double dx, dy;

    uint32_t mx = (uint32_t)std::floor(x);
    uint32_t my = (uint32_t)std::floor(y);

    double len = 1.0f; //unit vector length

    if (mx < _mapPtr->width() && my < _mapPtr->height())
    {
        _mapPtr->value(mx, my, len);
    }

    bool wallFound = false;

    while (!wallFound)
    {
        double current_x = x + len * cos(yawRad);
        double current_y = y + len * sin(yawRad);
        mx = (uint32_t)std::floor(current_x);
        my = (uint32_t)std::floor(current_y);

        if (mx >= _mapPtr->width() || my >= _mapPtr->height())
        {
            dx = x - current_x;
            dy = y - current_y;
            res = sqrt(dx * dx + dy * dy);
            return false;
        }

        uint8_t val{0};
        if (!_mapPtr->value(mx, my, val))
        {
            std::cout << "Error getting map value" << std::endl;
        }

        double tmpLen;

        _mapPtr->value(mx, my, tmpLen);

        len += tmpLen;

        if (val || tmpLen < 2.0)
        {
            dx = x - current_x;
            dy = y - current_y;

            res = sqrt(dx * dx + dy * dy);
            wallFound = true;
        }
    }

    return true;
}