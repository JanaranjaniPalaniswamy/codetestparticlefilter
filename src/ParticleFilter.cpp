#include "ParticleFilter.h"
#include <time.h>
#include <random>
#include <cstring>

void ParticleFilter::init(std::shared_ptr<Sensor> sensor, std::shared_ptr<Map> map, uint32_t numberOfParticles)
{
    _filterIteration = 0;
    _priorSetIndex = 0;
    _updatedSetIndex = 1;
    _particles[0].resize(numberOfParticles);
    _particles[1].resize(numberOfParticles);
    _accumulatedWeights.resize(numberOfParticles);
    _mapPtr = map;
    _sensorPtr = sensor;

    srand((unsigned int)time(NULL));

    for (uint32_t i = 0; i < numberOfParticles; i++)
    {
        double x = (double)(rand() % _mapPtr->width());
        double y = (double)(rand() % _mapPtr->height());
        double yaw = (double)(rand() % 360) * M_PI / 180.0;
        _particles[_priorSetIndex][i].init(sensor, x, y, yaw);
        _particles[_updatedSetIndex][i].init(sensor, x, y, yaw);
    }

    //_particles[_priorSetIndex][0].setPose(113.0, 114.0, 0);

    _dbgImgHandler = *map;
}

void ParticleFilter::filter(double dx, double dy, double dYaw, const double *measurement, uint32_t lengh)
{

     ////////////////////////////////////////////////////////////////////////////
    //////////YOUR CODE GOES HERE!!!////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
}

void ParticleFilter::writeParticlesToMapImage(const char *filename)
{
    _dbgImgHandler.updateImage(_mapPtr->imageData(), _mapPtr->imageSizeInByte());

    for (int i = 0; i < _particles[_priorSetIndex].size(); i++)
    {
        uint32_t x = (uint32_t)std::round(_particles[_priorSetIndex][i].x());
        uint32_t y = (uint32_t)std::round(_particles[_priorSetIndex][i].y());
        _dbgImgHandler.updateImage(x, y, 255, 0, 0);
    }

    _dbgImgHandler.saveImageAs(filename);
}

const double *ParticleFilter::mean() const
{
    return _mean;
}