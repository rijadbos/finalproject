/*
This file serves to store all the 
configuration settings of the I2C system
*/

/*====HEADERS=====*/
#include "projectheader.h"
/*====DEFINITIONS====*/

    /*====I2C SETTINGS for MPU====*/
    #define I2C_MASTER_SCL_IO      18    // GPIO pin for I2C Clock (SCL)
    #define I2C_MASTER_SDA_IO      8    // GPIO pin for I2C Data (SDA)
    #define I2C_MASTER_NUM         I2C_NUM_0
    #define I2C_MASTER_FREQ_HZ     100000  
    #define I2C_TIMEOUT_MS         1000
    #define SLAVE_ADDR             0x68    
    static const char *TAG = "i2c-master-example";

/*====FUNCTIONS DEFINITIONS=====*/
void i2c_configuration()
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
    ESP_LOGI(TAG, "I2C master initialized...");
}

esp_err_t i2c_send_data(uint8_t device_addr, uint8_t start_reg, uint8_t *buffer, size_t length);
