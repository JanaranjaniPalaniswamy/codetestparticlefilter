#ifndef _IMAGE_HANDLER_H_
#define _IMAGE_HANDLER_H_ _IMAGE_HANDLER_H_

#include <cstdint>

/*
*This class provides functionality to load, save, and manipulate BMP image data.
*Only 24-bit BMP images with zero compression are supported at the moment.
*Methods to request relevant image data are available.
*/

class ImageHandler
{
public:
    static constexpr uint8_t HEADER_SIZE = 54;

    ImageHandler(void);

    ImageHandler(const ImageHandler &other);

    ~ImageHandler();

    /*
    *Load an image given the filename.
    *@param filename the filename including its path.
    *@return true if loading was successful.
    */
    bool init(const char *filename);

    /*
    *Get the 54 bytes header of the bmp image
    */
    const uint8_t *header() const;
    /*
    *Get the raw image data
    */
    const uint8_t *imageData() const;
    /*
    *The bmp spec requires the width to be a 
    *multiple of 4 bytes. Thus, padding with
    *zeros might be required. This function
    *provides the padded width of the image.
    */
    uint32_t paddedWidth() const;
    /*
    *Get width of the image in pixel.
    */
    uint32_t width() const;
    /*
    *Get height of the image in pixel.
    */
    uint32_t height() const;
    /*
    *Get bits per pixel. Currently, only
    *24 bits are supported.
    */
    uint16_t bitsPerPixel() const;
    /*
    *Get offset between header and image data.
    */
    uint32_t offset() const;
    /*
    *Get the size of the image payload data.
    */
    uint32_t imageSizeInByte() const;
    /*
    *The bmp spec supports up to 14 compression methods in total.
    *Here, only non-compression (i.e. compression=0) is supported.
    */
    uint32_t compression() const;

    ImageHandler &operator=(const ImageHandler &other);

    /*
    *Set an rgb pixel in the image.
    *@param w x-pixel value
    *@param h y-pixel value
    *@param r the red byte
    *@param g the green byte
    *@param b the blue byte
    */

    void updateImage(uint32_t w, uint32_t h, uint8_t r, uint8_t g, uint8_t b);

    /*
    *Replace the complete image or only a part of it.
    *@param imgData the pointer with the payload data.
    *@param length the number of bytes to be replaced.
    */
    void updateImage(const uint8_t *imgData, uint32_t lenght);
    /*
    *Save your bmp image.
    *@param filename the image file name including its path.
    *@return true if saving the image was successful.
    */
    bool saveImageAs(const char *filename) const;

protected:
    uint8_t _header[54];
    uint32_t _paddedWidth;
    uint32_t _width;
    uint32_t _height;
    uint16_t _bitsPerPixel;
    uint32_t _offset;
    uint32_t _imgSizeByte;
    uint32_t _compression;

    uint8_t *_rawImageData{nullptr};
};

#endif