#include "ImageHandler.h"
#include <stdio.h>
#include <iostream>
#include <cassert>
#include <cstring>
#include <cmath>

ImageHandler::ImageHandler() {}

ImageHandler::ImageHandler(const ImageHandler &other)
{
    std::memcpy(_header, other.header(), HEADER_SIZE * sizeof(uint8_t));
    _paddedWidth = other.paddedWidth();
    _width = other.width();
    _height = other.height();
    _bitsPerPixel = other.bitsPerPixel();
    _offset = other.offset();
    _imgSizeByte = other.imageSizeInByte();
    _compression = other.compression();

    if (_rawImageData != nullptr)
    {
        delete[] _rawImageData;
    }

    int size = _paddedWidth * _height;
    _rawImageData = new uint8_t[size];
    std::memcpy(_rawImageData, other.imageData(), size * sizeof(uint8_t));
}

ImageHandler::~ImageHandler()
{
    if (_rawImageData != nullptr)
    {
        delete[] _rawImageData;
    }
}

bool ImageHandler::init(const char *filename)
{
    FILE *f{nullptr};
    f=fopen(filename, "rb");

    if (!f)
    {
        return false;
    }

    // read the 54-byte header
    fread(_header, sizeof(unsigned char), HEADER_SIZE, f);

    // extract image height and width from header
    _width = *(uint32_t *)&_header[18];
    int height = *(int *)&_header[22];

    //Due to bmp spec, height can be negative. Currently, we only support 'bottom up' line storage
    if (height < 0)
    {
        return false;
    }

    _height = (uint32_t)height;

    _bitsPerPixel = *(uint16_t *)&_header[28];

    assert(_bitsPerPixel == 24); //We assume 3 bytes per pixel. I don't want to implement the whole support..

    _offset = *(uint32_t *)&_header[10];

    assert(_offset == HEADER_SIZE);

    _imgSizeByte = *(uint32_t *)&_header[34];
    _compression = *(uint32_t *)&_header[30];

    _paddedWidth = (uint32_t)std::ceil((float)(_width * _bitsPerPixel) / 32.f) * 4; //Zero padding is applied if the bytes per row are not a multiple of 4

    uint32_t size = _paddedWidth * _height;

    if (_rawImageData != nullptr)
    {
        delete[] _rawImageData;
        _rawImageData = nullptr;
    }

    //uint8_t rawImgData[400];
    _rawImageData = new uint8_t[size];

    // read the rest of the data at once
    fread(&_rawImageData[0], sizeof(uint8_t), size, f);
    fclose(f);

    return true;
}

ImageHandler &ImageHandler::operator=(const ImageHandler &other)
{
    std::memcpy(_header, other.header(), HEADER_SIZE * sizeof(uint8_t));
    _paddedWidth = other.paddedWidth();
    _width = other.width();
    _height = other.height();
    _bitsPerPixel = other.bitsPerPixel();
    _offset = other.offset();
    _imgSizeByte = other.imageSizeInByte();
    _compression = other.compression();

    if (_rawImageData != nullptr)
    {
        delete[] _rawImageData;
    }

    int size = _paddedWidth * _height;
    _rawImageData = new uint8_t[size];
    std::memcpy(_rawImageData, other.imageData(), size * sizeof(uint8_t));

    return *this;
}

void ImageHandler::updateImage(uint32_t w, uint32_t h, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t i = w * 3;
    uint32_t j = _height - h - 1;
    uint32_t ptrIndex = j * _paddedWidth + i;

    if (ptrIndex < _imgSizeByte)
    {
        _rawImageData[ptrIndex] = b;
        _rawImageData[ptrIndex + 1] = g;
        _rawImageData[ptrIndex + 2] = r;
    }
}

void ImageHandler::updateImage(const uint8_t *imgData, uint32_t lenght)
{
    if (lenght <= _imgSizeByte)
    {
        std::memcpy(_rawImageData, imgData, sizeof(uint8_t) * lenght);
    }
}

bool ImageHandler::saveImageAs(const char *filename) const
{
    FILE *f{nullptr};
    f=fopen(filename, "wb");

    if (!f)
    {
        return false;
    }

    if (fwrite(_header, sizeof(uint8_t), HEADER_SIZE, f) < HEADER_SIZE)
    {
        std::cout << "Error writing bmp header" << std::endl;
        fclose(f);
        return false;
    }

    if (fwrite(_rawImageData, sizeof(uint8_t), _imgSizeByte, f) < _imgSizeByte)
    {
        std::cout << "Error writing bmp data" << std::endl;
        fclose(f);
        return false;
    }

    fclose(f);

    return true;
}

const uint8_t *ImageHandler::header() const { return _header; };
const uint8_t *ImageHandler::imageData() const { return _rawImageData; }
uint32_t ImageHandler::paddedWidth() const { return _paddedWidth; };
uint32_t ImageHandler::width() const { return _width; };
uint32_t ImageHandler::height() const { return _height; };
uint16_t ImageHandler::bitsPerPixel() const { return _bitsPerPixel; };
uint32_t ImageHandler::offset() const { return _offset; };
uint32_t ImageHandler::imageSizeInByte() const { return _imgSizeByte; };
uint32_t ImageHandler::compression() const { return _compression; };