/*
 * Copyright (c) 2022, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "pca9685_i2c_hal.h"

/* Hardware Specific Components */

/*  I2C User Defines  */
#define I2C_MASTER_SDA_IO 21 //gpio pin for sda
#define I2C_MASTER_SCL_IO 22 //gpio pin for scl
#define I2C_MASTER_FREQ_HZ 1000 //herz
#define I2C_MASTER_NUM              I2C_NUM_0



i2c_master_bus_handle_t bus_handle;

i2c_master_dev_handle_t dev_handle;


int16_t pca9685_i2c_hal_init(uint16_t addr_dev)
{
    int16_t err = PCA9685_OK;

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, //I2C_MASTER_FREQ_HZ
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_dev,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));


    return err == PCA9685_OK ? PCA9685_OK :  PCA9685_ERR;
}

int16_t pca9685_i2c_hal_read(uint8_t address, uint8_t *reg, uint8_t *data, uint16_t count)
{
    int16_t err = PCA9685_OK;


    i2c_master_receive(dev_handle, data, count, -1);


    return err == PCA9685_OK ? PCA9685_OK :  PCA9685_ERR;
}

int16_t pca9685_i2c_hal_write(uint8_t address, uint8_t *data, uint16_t count)
{
    int16_t err = PCA9685_OK;

    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, count, -1));


    return err == PCA9685_OK ? PCA9685_OK :  PCA9685_ERR;
}

void pca9685_i2c_hal_ms_delay(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);

}
