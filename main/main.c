/**********************************************************************************
 * 
 * 
 *  Copyright 2025 Alif Ilhan https://github.com/alifilhan0
 *  This is the firmware for esp32s3 sensor daughter board.
 *  TODO:- Calibrate and debug.
 * 
 * 
 **********************************************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "tca9548.h"
#include "esp_log.h"
#include "vl53l0x.h"
#include "vl53l1x.h"
#include "ultrasonic.h"
#include "tcs34725.h"
#define TAG "MAIN"

// Master side (ESP32 -> TCA9548 + sensors)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_FREQ_HZ 400000

// Slave side (ESP32 <- external board)
#define I2C_SLAVE_NUM I2C_NUM_1
#define I2C_SLAVE_SCL_IO 10
#define I2C_SLAVE_SDA_IO 11
#define I2C_SLAVE_ADDR 0x28
#define I2C_SLAVE_RX_BUF_LEN 128
#define I2C_SLAVE_TX_BUF_LEN 128

#define MAX_ULTRASONIC_SENSOR_NUM 4
#define MAX_TOF_NUM 4
#define RGB_CHANNEL 8
#define SECOND_MUX 0

//Sensor descriptors and configurations
static vl53l0x_t *vl53l0x[MAX_TOF_NUM];
static i2c_dev_t *dev;
static ESP32_TCS34725 *tcs34725;
static vl53l1x_t *vl53l1x[MAX_TOF_NUM];

static ultrasonic_sensor_t ultra[MAX_ULTRASONIC_SENSOR_NUM] = {
    { .trigger_pin = GPIO_NUM_1, .echo_pin = GPIO_NUM_11 },
    { .trigger_pin = GPIO_NUM_2, .echo_pin = GPIO_NUM_12 },
    { .trigger_pin = GPIO_NUM_3, .echo_pin = GPIO_NUM_13 },
    { .trigger_pin = GPIO_NUM_4, .echo_pin = GPIO_NUM_14 },

};

// Buffer to hold sensor data for external master
static uint16_t rx_buffer[16];





static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void i2c_slave_init(void)
{
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode,
                                       I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
}

int sensor_init()
{
    int i=0, ret=0;
    
    tca9548_init_desc(dev, 0x70, I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    for(i = 0; i < MAX_TOF_NUM; i++)
    {
        tca9548_set_channels(dev, i);
        vl53l0x[i] = vl53l0x_config(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, -1, 0x29 + i, 00);
        ret = vl53l0x_init(&vl53l0x[i]);
        if(ret)
        {
            ESP_LOGE("vl53l0x_init failed with error code -%d", ret);
        }
        vl53l0x_stopContinuous(vl53l0x[i]);
        vl53l0x_startContinuous(vl53l0x[i], 20000);
        
    }

    for(i = 0; i < MAX_ULTRASONIC_SENSOR_NUM; i++)
    {
        ret = ultrasonic_init(&ultra[i]);
        if(ret)
        {
            ESP_LOGE("ultrasonic_init failed with error code -%d", ret);
        }

    }

    
    for(i = 0; i < MAX_TOF_NUM; i++)
    {
        tca9548_set_channels(dev, 4 + i);
        vl53l1x[i] = vl53l1x_config(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, -1, 0x29 + i, 00);
        ret = vl53l1x_init(&vl53l0x[i]);
        if(ret)
        {
            ESP_LOGE("vl53l0x_init failed with error code -%d", ret);
        }
        vl53l1x_stopContinuous(vl53l0x[i]);
        vl53l1x_startContinuous(vl53l0x[i], 20000);
        
    }

    TCS_init(tcs34725, I2C_MASTER_NUM);

}

static void slave_task(void *arg)
{
    uint16_t tx_buffer;
    uint16_t rx_buffer[16];
    /* This rx_buffer is suppposed to store sensor data and will be sent back to master board.
        rx_buffer[0] - rx_buffer[3] will have ToF sensor data from right to behind in clockwise order
        rx_buffer[4] - rx_buffer[7] will have data read from ultrasonic sensors
        rx_buffer[8] - rx_buffer[10] will have red, green and blue respectiively data read from tcs34725 rgb sensor.
        rx_buffer[9] - rx_buffer[15] is reserved for flexible adaptation for alternative sensor debugging without 
        dropping any current sensor
    */
   int16_t distance;
   int i; 
    while (1) {
        // Check if master sent a request
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, tx_buffer, sizeof(tx_buffer), 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "Master requested data (cmd=%02x)", tx_buffer);

            switch(tx_buffer)
            {
                case 0x01: //Triggers a read from all available TOF sensors
                    for(i = 0; i < 4; i++)
                    {
                        tca9548_set_channel(dev, i);
                        rx_buffer[i] = vl53l0x_readRangeContinuousMillimeters(vl53l0x[i]);
                    }
                    break;
                case 0x02: //Triggers a read from all ultrasonic sensors
                    for(i = 0; i < MAX_ULTRASONIC_SENSOR_NUM; i++)
                    {
                        ultrasonic_measure_cm_temp_compensated(&ultra[i], 4, &distance, 25);
                        rx_buffer[4 + i] = distance;
                    }
                    break;
                case 0x03:
                    TCS_getRGB(tcs34725, &rx_buffer[8], &rx_buffer[9], &rx_buffer[10]);
                    break;
                    //This is a reserved function for sensor testing and calibration.
                case 0x04: //Triggers a read from all available TOF sensors
                    for(i = 0; i < 4; i++)
                    {
                        tca9548_set_channel(dev, 4 + i);
                        rx_buffer[11 + i] = vl53l1x_readRangeContinuousMillimeters(vl53l1x[i]);
                    }
                    break;
                default:
                    ESP_LOGI(TAG, "Wrong input");
                    break;

            }
        }
    }
}

void app_main(void)
{
    i2c_master_init();
    i2c_slave_init();
    sensor_init();
    // Start background tasks
    xTaskCreate(slave_task,  "slave_task",  4096, NULL, 5, NULL);
}
