#ifndef _SENSOR_H_
#define _SENSOR_H_ _SENSOR_H

#define _USE_MATH_DEFINES

#include "Map.h"
#include <cmath>
#include <math.h>
#include <memory>

class Sensor
{
public:
    static constexpr uint32_t NUM_SENSOR_MEASUREMENTS = 180;
    static constexpr double MEASUREMENT_STEP = 0.5;
    static constexpr double ANGULAR_RESOLUTION = 1.0 * M_PI / 180.0;

    Sensor(std::shared_ptr<Map> mapPtr);
    /*
    *Set a new sensor pose.
    *@param x new x-pose
    *@param y new y-pose
    *@param yawRad new yaw angle in [rad]
    */
    void updatePose(double x, double y, double yawRad);

    /*
    *Perform a simulated lidar measurement given the map pointer.
    *The assumption is that the lidar sweeps from -90° to +90° (180° in total)
    *with an angular delta of ANGULAR_RESOLUTION between the beams.
    *@return the measurement array of length NUM_SENSOR_MEASUREMENTS.
    */

    const double *doMeasurement();

private:
    /*
     * Perform a simple ray tracing algorithm, following the beam direction cell by cell
     * with a beam lengt delta of MEASUREMENT_STEP. More accurate than doRaytracingFast but
     * horrible slow.
     * @param x the x-position were to start the ray tracing
     * @param y the y-position were to start the ray tracing
     * @param yawRad the direction of the beam [rad]
     * @param res the result is stored here
     * @param true if the the beam hit a wall inside of the map boundaries
     */
    bool doRaytracing(double x, double y, double yawRad, double &res);

    /*
    * Perform a ray tracing operation based on the pre-computed distance map. This 
    * operation is much faster than doRaytracing.
    * @param x the x-position were to start the ray tracing
    * @param y the y-position were to start the ray tracing
    * @param yawRad the direction of the beam [rad]
    * @param res the result is stored here
    * @param true if the the beam hit a wall inside of the map boundaries
    */
    bool doRaytracingFast(double x, double y, double yawRad, double &res);

    std::shared_ptr<Map> _mapPtr{nullptr};
    double _sensorPosX{0};
    double _sensorPosY{0};
    double _sensorYaw{0};
    double _measurement[NUM_SENSOR_MEASUREMENTS];
};

#endif