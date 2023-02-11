#ifndef __OBJECT_DETECT_CAMERA_IMAGE_CONVERTER_H_
#define __OBJECT_DETECT_CAMERA_IMAGE_CONVERTER_H_

#include "ros2_definitions.h"

#include <jetson-utils/cudaUtility.h>
#include <jetson-utils/imageFormat.h>



/**
 * @GPU image conversion
 * 
 */

class imageConverter{
public:
    /**
     * @Output image pixel type
     * 
     */

    typedef uchar3 PixelType;

    /**
     * @ Image format used  for internal CUDA processing
     * 
     */

    static const imageFormat InternalFormat = IMAGE_RGB8;

    /**
     * @Image format used for outputting ROS Image messages
     * 
     * 
     */

    static const imageFormat ROSOutputFormat = IMAGE_BGR8;

    /* Constructor */

    imageConverter();

    /* Destructor */

    ~imageConverter();

    /* Free memory */

    void Free();

    /* Converter 32-bit RGBA floating point */

    bool Convert(const sensor_msgs::msg::Image::ConstSharedPtr& input);

    /* Convert to ROS sensor_msgs::Image message */
    bool Convert(sensor_msgs::msg::Image& msg_out, imageFormat outputFormat);

    /* Convert to ROS sensor_msgs::IMage message */
    bool Convert( sensor_msgs::msg::Image& msg_out, imageFormat outputFormat, PixelType* imageGPU);

    /* resize memory if necessary */

    bool Resize(uint32_t width, uint32_t height, imageFormat inputFormat);

    /**
     * @brief  Retrieve the converted image width
     * 
     */

    inline uint32_t GetWidth() const {return mWidth;}

    /**
     * @brief Retrieve the converted image height
     * 
     */

    inline uint32_t GetHeight() const {return mHeight;}

    /**
     * @brief Retrieve the GPU pointyer of the converted image
     * 
     */

    inline PixelType* ImageGPU() const  { return mOutputGPU; }

private:
    uint32_t mWidth;
    uint32_t mHeight;
    size_t mSizeInput;
    size_t mSizeOutput;

    void* mInputCPU;
    void* mInputGPU;

    PixelType* mOutputCPU;
    PixelType* mOutputGPU;
};



#endif // __MACHINE_LEARNING_PKG_IMAGE_CONVERTER_H_