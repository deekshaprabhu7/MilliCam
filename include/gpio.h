

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100 //LED Blink interval

#define FRAME_VALID_PIN 24 //gpio0 //gpio pin definition
#define TEST_PIN 25

#define I2C1_NODE DT_NODELABEL(mysensor) //i2c1


#define MCLK 26 //gpio0 //gpio pin definition

/** @brief Symbol specifying ouput pin associated with the task.  **/
#define OUTPUT_PIN MCLK


extern const struct device *gpio0;
extern const struct device *gpio1;