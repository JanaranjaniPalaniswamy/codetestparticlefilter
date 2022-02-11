#include "Map.h"
#include <limits.h>
#include <algorithm>
#include <cmath>
#include <cstring>

Map::~Map()
{
    if (_data)
    {
        delete[] _data;
        _data = nullptr;
    }

    if (_distData != nullptr)
    {
        delete[] _distData;
        _distData = nullptr;
    }
}

bool Map::init(const char *filename)
{
    std::cout << "Loading image..." << std::endl;
    ImageHandler::init(filename);

    if (_data != nullptr)
    {
        delete[] _data;
    }

    if (_distData != nullptr)
    {
        delete[] _distData;
    }
    _data = new uint8_t[_width * _height];
    _distData = new double[_width * _height];

    std::memset(_data, 0, sizeof(uint8_t) * _width * _height);

    for (uint32_t i = 0; i < _width * _height; i++)
    {
        _distData[i] = std::numeric_limits<double>::max() - 10.0;
    }

    for (uint32_t j = 0; j < _height; j++)
    {
        for (uint32_t i = 0; i < _paddedWidth - 3; i += 3)
        {
            uint8_t r = _rawImageData[j * _paddedWidth + i];
            uint8_t g = _rawImageData[j * _paddedWidth + i + 1];
            uint8_t b = _rawImageData[j * _paddedWidth + i + 2];

            if (r < 255 || g < 255 || b < 255) //Here we have potential obstacles in the map
            {
                uint32_t pix_w = i / 3;
                uint32_t pix_h = _height - j - 1; //BMP images store the data 'bottom up', i.e., the last line is being read first

                _data[pix_h * _width + pix_w] = 1; //Create a wall for the robot
                _distData[pix_h * _width + pix_w] = 0.0;
            }
        }
    }

    //Draw walls as artificial barriers for the particles and for raytracing
    drawOuterWalls();

    std::cout << "Computing distance trafo.." << std::endl;

    computeDistMap();

    std::cout << "Done" << std::endl;

    return true;
}

bool Map::value(uint32_t x, uint32_t y, uint8_t &val) const
{
    uint32_t index = y * _width + x;
    if (index < _width * _height)
    {
        val = _data[index];
        return true;
    }
    return false;
}

bool Map::value(uint32_t x, uint32_t y, double &val) const
{
    uint32_t index = y * _width + x;
    if (index < _width * _height)
    {
        val = _distData[index];
        return true;
    }
    return false;
}

void Map::drawOuterWalls()
{
    for (uint32_t i = 0; i < _width; i++)
    {
        _data[i] = 1;
        _distData[i] = 0.0;
        _data[_width * (_height - 1) + i] = 1;
        _distData[_width * (_height - 1) + i] = 0.0;
    }

    for (uint32_t i = 0; i < _height; i++)
    {
        _data[i * _width] = 1;
        _distData[i * _width] = 0.0;
        _data[i * _width + _width - 1] = 1;
        _distData[i * _width + _width - 1] = 0.0;
    }
}

void Map::computeDistMap()
{

    auto getMinElement = [this](int i, int j) -> double
    {
        double sqrt_two = sqrt(2.0);

        double min = std::numeric_limits<double>::max() - 10.0;

        for (int m = -1; m <= 1; m++)
        {
            for (int n = -1; n <= 1; n++)
            {
                if (i + m >= 0 && i + m < (int)_width && j + n >= 0 && j + n < (int)_height)
                {
                    double dist = (m != 0 || n != 0) ? 1.0 : 0.0;
                    if (m != 0 && n != 0)
                    {
                        dist = sqrt_two;
                    }
                    _distData[(j + n) * _width + i + m] + dist < min ? min = _distData[(j + n) * _width + i + m] + dist : min;
                }
            }
        }

        return min;
    };

    //Do forward propagation
    for (uint32_t j = 0; j < _height; j++)
    {
        for (uint32_t i = 0; i < _width; i++)
        {

            double min = getMinElement(i, j);

            _distData[j * _width + i] = min;

            //std::cout << i << " " << j << " " << _distData[j * _width + i] << std::endl;
        }
    }

    //Then do backward propagation

    for (int j = _height - 1; j >= 0; j--)
    {
        for (int i = _width - 1; i >= 0; i--)
        {

            double min = getMinElement(i, j);

            _distData[j * _width + i] = min;

            //std::cout << i << " " << j << " " << _distData[j * _width + i] << std::endl;
        }
    }

    /*
    for (int i = 0; i < _height; i++)
    {
        for (int j = 0; j < _width; j++)
        {
            std::cout << _distData[i * _width + j] << "      ";
        }

        std::cout << std::endl;
    }*/

    //dbgSaveDistanceMap();
}

void Map::dbgSaveDistanceMap()
{
    ImageHandler distImgHndl = *this;

    double min = 10000000000;
    double max = 0.0;

    for (uint32_t i = 0; i < _width * _height; i++)
    {
        if (_distData[i] < min)
        {
            min = _distData[i];
        }

        if (_distData[i] > max)
        {
            max = _distData[i];
        }
    }

    uint8_t *distImg = new uint8_t[_imgSizeByte];
    std::memset(distImg, 0, _imgSizeByte);

    for (uint32_t j = 0; j < _height; j++)
    {
        for (uint32_t i = 0; i < _width; i++)
        {
            double val = _distData[j * _width + i];
            val = (val - min) / (max - min);

            assert(val >= 0.0);

            uint32_t grayUint32 = (uint32_t)(val * 255);

            assert(grayUint32 <= 255);

            if (grayUint32 > 255)
            {
                grayUint32 = 255;
            }

            uint8_t gray = (uint8_t)grayUint32;

            uint32_t m = i * 3;
            uint32_t n = _height - j - 1;
            uint32_t ptrIndex = n * _paddedWidth + m;

            if (ptrIndex < _imgSizeByte)
            {
                distImg[ptrIndex] = gray;
                distImg[ptrIndex + 1] = gray;
                distImg[ptrIndex + 2] = gray;
            }
        }
    }

    FILE *f{nullptr};
    f = fopen("C:/dev/devstack/codetestparticlefilter/distImg.bmp", "wb");

    if (!f)
    {
        return;
    }

    if (fwrite(_header, sizeof(uint8_t), HEADER_SIZE, f) < HEADER_SIZE)
    {
        std::cout << "Error writing bmp header" << std::endl;
        fclose(f);
        return;
    }

    if (fwrite(distImg, sizeof(uint8_t), _imgSizeByte, f) < _imgSizeByte)
    {
        std::cout << "Error writing bmp data" << std::endl;
        fclose(f);
        return;
    }

    fclose(f);

    delete[] distImg;
}
