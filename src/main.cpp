#include "ParticleFilter.h"

//You must use your own path here!
static const char *MAP_FILE_PATH = "YOUR_FILE_PATH/Map.bmp";
static const char *MEASUREMENT_FILE_PATH = "YOUR_FILE_PATH/measurements.txt";

//#define _CREATE_YOUR_OWN_MEASUREMENT_FILE_ _CREATE_YOUR_OWN_MEASUREMENT_FILE_

void createMeasurementFile(const char *filename, Robot &robot)
{
    FILE *f{nullptr};

    f = fopen(filename, "w");

    if (!f)
    {
        return;
    }

    double dx = 10.0;
    for (double x = robot.x(); x < 450; x += dx)
    {
        robot.move(dx, 0.0, 0.0);
        const double *measurement = robot.perceiveEnvironment();
        fprintf(f, "%lf %lf %lf %lf %lf %lf ", robot.x(), robot.y(), robot.yaw(), dx, 0.0, 0.0);
        for (uint32_t i = 0; i < robot.totalNumberOfMeasurements(); i++)
        {
            fprintf(f, "%lf ", measurement[i]);
        }

        fprintf(f, "\n");
    }

    fclose(f);
}

/*
*Use this method to load the measurement file and read the measurements line by line.
*For every sensor reading call the particle filter to update the posterior.
*@param filename the path to the file containing the measurements
*@param filter a reference to the particle filter
*@param sensorPtr a pointer to the sensor; use this if required (e.g. to get the number of measurements etc.)
*/
void executeParticleFilter(const char *filename, ParticleFilter &filter, std::shared_ptr<Sensor> sensorPtr)
{

    ////////////////////////////////////////////////////////////////////////////
    //////////YOUR CODE GOES HERE!!!////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
}

int main()
{
    std::shared_ptr<Map> mapPtr = std::make_shared<Map>();

    /***************************************
    ***Adapt your file path accordingly!****
    ****************************************
    */
    mapPtr->init(MAP_FILE_PATH);

    std::shared_ptr<Sensor> sensorPtr = std::make_shared<Sensor>(mapPtr);

#ifdef _CREATE_YOUR_OWN_MEASUREMENT_FILE_

    Robot robot;
    robot.init(sensorPtr, 113.0, 114.0, 0.0);

    /***************************************
    ***Adapt your file path accordingly!****
    ****************************************
    */

    createMeasurementFile(MEASUREMENT_FILE_PATH, robot);

#endif

    ParticleFilter filter;
    filter.init(sensorPtr, mapPtr, 10000);

    /***************************************
    ***Adapt your file path accordingly!****
    ****************************************
    */

    executeParticleFilter(MEASUREMENT_FILE_PATH, filter, sensorPtr);
}
