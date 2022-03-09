#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_ _PARTICLE_FILTER_H_
#include "Particle.h"
#include <vector>
#include <random>

using std::vector;

class ParticleFilter
{
public:
    /*
    *Sample particle filter from a normal distribution
    *define by the following standard deviations.
    *Adapt those values if you like.
    */
    static constexpr double stdDevX = 2.0;
    static constexpr double stdDevY = 2.0;
    static constexpr double stdDevYaw = 5.0 * M_PI / 180.0;

    typedef std::vector<Particle> ParticleSet;

    /*
    *Init the particle filter by providing access to the sensor and the map. The sensor will be shared
    *by all particles. You can use the map to get access to its boundaries.
    *@param sensor
    *@param map
    *@param numberOfParticles define the number of particles used to approximate the posterior distribution
    */
    void init(std::shared_ptr<Sensor> sensor, std::shared_ptr<Map> map, uint32_t numberOfParticles);
    /*
    *Compute the expected robot pose.
    *@return the pose [x,y,yaw] as an array
    */
    const double *mean() const;

    /*
    *This method conducts a single filter step.
    *@param dx the robot motion in x-direction
    *@param dy the robot motion in y-direction
    *@param dYaw the heading delta which results from the robot motion
    *@param measurement a sensor measurement entries
    *@param length the number of measurement entries
    */
    void filter(double dx, double dy, double dYaw, const double *measurement, uint32_t length, vector<double> &gt);

    /*
    * Num of particles 
    */
    const uint32_t NUM_OF_PARTICLES = 10000;

    /*
    * Motion model parameters
    */
    //static constexpr double ALPHA_1 = 0.01;
    //static constexpr double ALPHA_2 = 0.01;
    //static constexpr double ALPHA_3 = 0.1;
    //static constexpr double ALPHA_4 = 0.1;

private:
    /*
    *Nice feature to display the particle set in a BMP image
    *defined by the filename. Thus, you can print the
    *particle set for every filter iteration.
    *@param filename
    */
    void writeParticlesToMapImage(const char *filename);

    std::shared_ptr<Map> _mapPtr;
    //We need two particle set.
    //A predicted set and the posterior
    ParticleSet _particles[2];
    //Use _priorSetIndex and _updatedSetIndex
    //to mark predicted and posterior particle sets
    //if you want to
    uint8_t _priorSetIndex{0};
    uint8_t _updatedSetIndex{1};
    std::shared_ptr<Sensor> _sensorPtr;
    std::random_device _rd{};
    std::mt19937 _mersenne{_rd()};
    std::vector<double> _accumulatedWeights;
    double _mean[3];
    ImageHandler _dbgImgHandler;
    uint32_t _filterIteration{0};
};

#endif