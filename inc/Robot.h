#ifndef _ROBOT_H_
#define _ROBOT_H_
#include "Sensor.h"

/*
* This class provides an implementation of an artificial robot that can be
* moved around in its environment and also perceiving its world with its sensor.
* This class has been used to create the measurement file. Moreover, it is the
* base class for the particles as every particle can be interpreted as a robot.
*/

class Robot
{
public:
    /*
    *Set the initial pose of the robot as well as the sensor pointer.
    *A shared pointer is important as every particle (aka robot) in our particle filter
    *has access to the same memory holding the map.
    *@param sensor A shared pointer to the sensor. 
    *@param x the x position of the robot
    *@param y the y position of the robot
    *@param yawRad the heading of the robot [rad]
    */
    void init(std::shared_ptr<Sensor> sensor, double x, double y, double yawRad);

    /*
    *Move the robot by defining a delta pose.
    *@param dx the delta in x direction
    *@param dy the delta in y direction
    *@param dYaw the heading delta [rad] 
    */
    void move(double dx, double dy, double dYaw);

    /*
    *Get the number of measurement per sensor reading.
    */
    uint32_t totalNumberOfMeasurements();
    /*
    *Performs a sensor reading using the sensor object internally.
    *@return the array containing the measurement results.
    */
    const double *perceiveEnvironment();

    /*
    *Get the x position of the robot.
    */
    double x() const;

    /*
    *Get the y position of the robot.
    */
    double y() const;
    /*
    *Get the heading of the robot [rad].
    */
    double yaw() const;

protected:
    std::shared_ptr<Sensor> _sensor{nullptr};
    double _x{0};
    double _y{0};
    double _yaw{0};
};

#endif