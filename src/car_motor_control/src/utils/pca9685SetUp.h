#include "../../libraries/pca9685/pca9685.h"


class PCA9685SetUp {

public:
    PCA9685SetUp();
    ~PCA9685SetUp();

    void moveServo(int channel, float degrees);

private:
    // servo variables
    uint16_t pulse_width;  //microseconds 1 us = 1000 ns
    I2C_Driver *i2c_driver;
    const uint8_t pca_9685_address = 0x40;  //default address
    const char *i2c_device_name;
    PCA9685 *pca9685;
    float pwm_freq = 50.0; // hz
    //Servo functions
    void degrees_to_pulses(float degrees);
    void init_i2c_driver();
    void init_PCA9685_object();
    
    void close_PCA9685_object();
};