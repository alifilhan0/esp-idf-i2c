#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "tca9548.h"
#include "esp_log.h"
#include "vl53l0x.h"
#include "vl53l1x.h"
#include "ultrasonic.h"
#define TAG "MAIN"

// Master side (ESP32 → TCA9548 + sensors)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_FREQ_HZ 400000

// Slave side (ESP32 ← external board)
#define I2C_SLAVE_NUM I2C_NUM_1
#define I2C_SLAVE_SCL_IO 10
#define I2C_SLAVE_SDA_IO 11
#define I2C_SLAVE_ADDR 0x28
#define I2C_SLAVE_RX_BUF_LEN 128
#define I2C_SLAVE_TX_BUF_LEN 128

// Buffer to hold sensor data for external master
static uint8_t sensor_data[16];   // pretend we have up to 16 sensors

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

static void sensor_task(void *arg)
{
    while (1) {
        for (uint8_t ch = 0; ch < 4; ch++) {
            // Select channel on TCA9548
            if (tca9548_select_channel(I2C_MASTER_NUM, TCA9548_I2C_ADDR, ch) == ESP_OK) {
                // Here you would normally read from the real sensor
                // For demo, just fill with fake data
                sensor_data[ch] = ch * 10 + 5;
                ESP_LOGI(TAG, "Sensor[%d] = %d", ch, sensor_data[ch]);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void slave_task(void *arg)
{
    uint8_t rx_buffer[16];
    while (1) {
        // Check if master sent a request
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, rx_buffer, sizeof(rx_buffer), 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "Master requested data (cmd=%02x)", rx_buffer[0]);

            // For simplicity, always send back all sensor data
            i2c_slave_write_buffer(I2C_SLAVE_NUM, sensor_data, sizeof(sensor_data), 10 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    i2c_master_init();
    i2c_slave_init();

    // Start background tasks
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(slave_task,  "slave_task",  4096, NULL, 5, NULL);
}
