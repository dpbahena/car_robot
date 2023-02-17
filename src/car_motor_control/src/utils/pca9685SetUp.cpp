#include "pca9685SetUp.h"

PCA9685SetUp::PCA9685SetUp(){
    pulse_width = 1500;  // default pulse_width in us
    i2c_device_name = "/dev/i2c-8";
    init_i2c_driver();
    init_PCA9685_object();
}
PCA9685SetUp::~PCA9685SetUp(){
    close_PCA9685_object();
}

void PCA9685SetUp::init_i2c_driver(){
    
    i2c_driver = new I2C_Driver(i2c_device_name);
    bool openSuccess = i2c_driver->open_i2c_device();
    if(!openSuccess){
        printf("FAILED to open I2C driver");
    }
    else{
        printf("I2C Device file descriptor: %d",i2c_driver->get_file_descriptor());
    }
}

void PCA9685SetUp::init_PCA9685_object(){
   
    pca9685 = new PCA9685(i2c_driver,pca_9685_address);

    if(!pca9685->set_mode1_defaults()){
        printf("Failed - PCA9685 - set mode 1 defaults");
    }
    if(!pca9685->set_mode2_defaults_for_driving_servos()){
        printf("Mode 2 failed");
    }

    // Call wake up functions
    if(!pca9685->wakeup()){
        printf("PCA9685 failed to wake up");
    }
    // Get current mode 1 configuration
    uint8_t current_mode1;
    if(!pca9685->get_mode1(&current_mode1)){
        printf("PCA9685 failed mode 1 configuration");
    }else{
        std::cout << "PCA9685 - get mode 1 returned: " << std::bitset<8>(current_mode1) 
                                        << ", for I2C address " << static_cast<int>(pca9685->get_i2c_address()) << "\n" 
                                        << "                     expected: " << "00000100" << "\n";  
    }
    // Get current mode 2 Configuration
    uint8_t current_mode2;
    if(!pca9685->get_mode2(&current_mode2)){
        printf("PCA9685 failed mode 2 configuration"); 
    }else{
        std::cout << "PCA9685 - get mode 2 returned: " << std::bitset<8>(current_mode2) 
                                        << ", for I2C address " << static_cast<int>(pca9685->get_i2c_address()) << "\n"
                                        << "                     expected: " << "00000100" << "\n";     
    }

    // set the PWM frequency
    if(!pca9685->set_pwm_frequency_in_hz(pwm_freq)){
            printf("FAILED - PCA9685 - set the PWM frequency NOT successful for I2C address %d\n", pca9685->get_i2c_address());
    }

    // Get the frequency just to double check

    float pwm_freq_retrieved;
    uint8_t pre_scale_retrieved;
    
    if(!pca9685->get_pwm_frequency_in_hz_and_prescale(&pwm_freq_retrieved, &pre_scale_retrieved)){
        printf("PCA9685 failed to get Frequency for I2C address %d\n", pca9685->get_i2c_address());
    }else{
        printf("PCA9685 - get PWM frequency returned:\n> freq = %f\n> pre scale = %d\nfor I2C address %d\n", pwm_freq_retrieved, pre_scale_retrieved, pca9685->get_i2c_address() );
    }

     int channel_to_set;
     bool result;
    // set the frequency tp a channel (0-15)  but only channels 0 and 1 will be set
    for (channel_to_set = 0; channel_to_set < 2; channel_to_set++){
        result = pca9685->set_pwm_full_on_or_full_off(channel_to_set, false);
        if(!result){
            printf("PCA9685 not successfull for channel %d \n", channel_to_set);
        }
    }
    // > Get the PWM pulse setting
	uint16_t channel_0_on_count;
	uint16_t channel_0_off_count;
	
    for (channel_to_set = 0; channel_to_set < 2; channel_to_set++){
        result = pca9685->get_pwm_pulse(channel_to_set,&channel_0_on_count, &channel_0_off_count);
        if (result)
        {
            std::cout << "PCA9685 - get channel " << channel_to_set << " pulse returned:\n";
            std::cout << "channel on  = " << std::bitset<16>(channel_0_on_count) << "\n";
            std::cout << "    expected: " << "0000000000000000" << "\n";
            std::cout << "channel off = " << std::bitset<16>(channel_0_off_count) << "\n";
            std::cout << "    expected: " << "0001000000000000" << "\n";
        }
        else
        {
            printf("FAILED - PCA9685 - get channel %d pulse NOT successful for I2C address %d\n", channel_to_set, pca9685->get_i2c_address() );
        }

    }
}

void PCA9685SetUp::close_PCA9685_object(){
    
    usleep(1000000);
	printf("\n\n");
	printf("============================\n");
	printf("SET ALL CHANNELS TO FULL OFF\n");
    bool result;

	result = pca9685->set_all_channels_full_off();
	if (result)
	{
		printf("PCA9685 - set all channels to full off successfully, for I2C address %d\n", pca9685->get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - set all channels to full off NOT successful for I2C address %d\n", pca9685->get_i2c_address() );
	}

	printf("\n\n");

	// > Get the PWM pulse setting
	uint16_t channel_on_count;
	uint16_t channel_off_count;
    int channel_to_set;
    for (channel_to_set = 0; channel_to_set < 2; channel_to_set++){
        result = pca9685->get_pwm_pulse(channel_to_set,&channel_on_count, &channel_off_count);
        if (result)
        {
            std::cout << "PCA9685 - get channel " << channel_to_set << " pulse returned:\n";
            std::cout << "channel on  = " << std::bitset<16>(channel_on_count) << "\n";
            std::cout << "    expected: " << "0000000000000000" << "\n";
            std::cout << "channel off = " << std::bitset<16>(channel_off_count) << "\n";
            std::cout << "    expected: " << "0001000000000000" << "\n";
            std::cout << "on count: " << channel_on_count << "\n";
            std::cout << "off count: " << channel_off_count << "\n";

        }
        else
        {
            printf("FAILED - PCA9685 - get channel %d pulse NOT successful for I2C address %d\n", channel_to_set, pca9685->get_i2c_address() );
        }
    }
}


void PCA9685SetUp::moveServo(int channel, float degrees){
    degrees_to_pulses(degrees); // updates the pulse_width global variable
    // > Set the frequency
	//uint16_t pulse_width_in_us = 1500;

    bool result = pca9685->set_pwm_pulse_in_microseconds(channel,pulse_width);
    if (result)
    {
        printf("PCA9685 - set channel %d to %d micro seconds successfully, for I2C address %d\n", channel, pulse_width, pca9685->get_i2c_address() );
    }
    else
    {
        printf("FAILED - PCA9685 - set channel %d pulse NOT successful for I2C address %d\n", channel, pca9685->get_i2c_address() );
    }
    



    // Convert the pulse width to a count
	float micro_seconds_per_count = (1000000.0 / 4096.0) / pwm_freq;
	uint16_t expected_off_count = static_cast<uint16_t>( static_cast<float>(pulse_width) / micro_seconds_per_count );

	/// > Get the PWM pulse setting
	uint16_t channel_0_on_count;
	uint16_t channel_0_off_count;
    
    
  
    result = pca9685->get_pwm_pulse(channel,&channel_0_on_count, &channel_0_off_count);
    if (result)
    {
        std::cout << "PCA9685 - get channel " << channel << " pulse returned:\n";
        std::cout << "channel on  = " << std::bitset<16>(channel_0_on_count) << "\n";
        std::cout << "    expected: " << "0000000000000000" << "\n";
        std::cout << "channel off = " << std::bitset<16>(channel_0_off_count) << "\n";
        std::cout << "    expected: " << std::bitset<16>(expected_off_count)  << "\n";
        std::cout << "on count: " << channel_0_on_count << "\n";
        std::cout << "off count: " << channel_0_off_count << "\n";
    }
    else
    {
        printf("FAILED - PCA9685 - get channel %d pulse NOT successful for I2C address %d\n", channel, pca9685->get_i2c_address() );
    }
}

void PCA9685SetUp::degrees_to_pulses(float degrees){
    auto pulse_per_degree = 1000/90;
    pulse_width = 500 + degrees * pulse_per_degree;
    //pulse_width_in_us =1500;
    
}