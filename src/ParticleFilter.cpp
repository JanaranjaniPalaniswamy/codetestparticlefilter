#include "ParticleFilter.h"
#include <time.h>
#include <random>
#include <cstring>
#include <random>
#include <numeric>
#include <algorithm>
#include <vector>

using namespace std;
static const char* DEBUG_FILE_PATH = "C:/Users/Jhanaranjani/source/repos/codetestparticlefilter/test.bmp";


void ParticleFilter::init(std::shared_ptr<Sensor> sensor, std::shared_ptr<Map> map, uint32_t numberOfParticles)
{
    _filterIteration = 0;
    _priorSetIndex = 0;
    _updatedSetIndex = 1;
    _particles[0].resize(numberOfParticles); //resize the particleset to numofParticles
    _particles[1].resize(numberOfParticles);
    _accumulatedWeights.resize(numberOfParticles); //resize weight
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

void ParticleFilter::filter(double dx, double dy, double dYaw, const double *measurement, uint32_t length, vector<double> &gt)
{
    Particle p;
    double x_1, y_1, yaw_1;
    const double* new_measurement;
    double total_weight = 0.0;

    // Gaussian noise distribution with zero mean and standard deviation for each parameter
    normal_distribution<double> noise_x{0, stdDevX};
    normal_distribution<double> noise_y{0, stdDevY};
    normal_distribution<double> noise_yaw{0, stdDevYaw};

    // noise model
    default_random_engine gen;
     
     //For all the particles
    for (int i = 0; i < _particles[_priorSetIndex].size(); i++)
    {
        auto c = cos(_particles[_priorSetIndex][i].yaw() - gt[2]);
        auto s = sin(_particles[_priorSetIndex][i].yaw() - gt[2]);
        
        x_1 = _particles[_updatedSetIndex][i].x() + (c * dx - s * dy);
        y_1 = _particles[_updatedSetIndex][i].y() + (s * dx + c * dy);
        yaw_1 = _particles[_updatedSetIndex][i].yaw() + dYaw;

        x_1 += x_1 + noise_x(gen);
        y_1 += y_1 + noise_y(gen);
        yaw_1 += yaw_1 + noise_yaw(gen);

        //motion update
        p.setPose(x_1, y_1, yaw_1);       
        _particles[_updatedSetIndex][i].move(dx, dy, dYaw);

      
        p.computeWeight(measurement, length);

        _accumulatedWeights.push_back(p.weight());
        
        total_weight = accumulate(_accumulatedWeights.begin(), _accumulatedWeights.end(), 0);
    }

   
    //Normalize weight
    for (int k = 0; k < NUM_OF_PARTICLES; k++) {

        double weight = _accumulatedWeights[k]/total_weight;

        _accumulatedWeights.push_back(weight);
    }

    /*
    *Perform resampling
    */
    uniform_real_distribution<> dist(0.0, (double)(1.0 / (double)NUM_OF_PARTICLES));
    double r = dist(gen);
    double c = _accumulatedWeights[0];
    int j = 0;
    long M = NUM_OF_PARTICLES;

    for (int i = 1; i < _particles[_priorSetIndex].size(); i++)
    {
        double u = r + ((double)(i - 1) / (double)M);

        while (u > c)
        {
            ++j;
            double temp = _accumulatedWeights[i];
            c += temp;
        }

        _particles[_priorSetIndex][i - 1] = _particles[_updatedSetIndex][i];

    }

    writeParticlesToMapImage(DEBUG_FILE_PATH);
      
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