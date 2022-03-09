#include "ParticleFilter.h"
#include <fstream>
#include <sstream>
#include <algorithm>
using namespace std;

//You must use your own path here!
static const char *MAP_FILE_PATH = "C:/Users/Jhanaranjani/source/repos/codetestparticlefilter/Map.bmp";
static const char *MEASUREMENT_FILE_PATH = "C:/Users/Jhanaranjani/source/repos/codetestparticlefilter/measurements.txt";

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
   
    std::ifstream mfile(MEASUREMENT_FILE_PATH);
    string line;

    vector<double> ground_truth;
    int count = 0; //count of each line
    double dx = 0.0;
    double dy = 0.0;
    double dYaw = 0.0;
    double arr[(Sensor::NUM_SENSOR_MEASUREMENTS)+6];
    double measurements[Sensor::NUM_SENSOR_MEASUREMENTS];
    
    uint32_t length =0;

    if (!mfile)
    {
        std::cout << "couldnt open measurement file" << std::endl;
        return;
    }
    
    while (!mfile.eof() && count <34)
    {   
        //Read the measurements.txt file line by line
        if (getline(mfile, line))
        {
            stringstream str(line);
            for (int j = 0; j < 186; j++)
            {
                str >> arr[j];
            }
            count++;
        }

        //Store the data from each line ot variables and measurement array
        for (int i = 0; i < size(arr) / sizeof(arr[0]); i++)
        {
            if (i < 6)
            {
                ground_truth.push_back(arr[0]);
                ground_truth.push_back(arr[1]);
                ground_truth.push_back(arr[2]);
                dx = arr[3];
                dy = arr[4];
                dYaw = arr[5];
            }
            else {
                copy(begin(arr) + 6, end(arr), begin(measurements));
                length = sizeof(measurements) / sizeof(measurements[0]);
            }

        }
        //Execute Particle filter for each measurement data
        filter.filter(dx, dy, dYaw, measurements, length, ground_truth);
    }
  

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
