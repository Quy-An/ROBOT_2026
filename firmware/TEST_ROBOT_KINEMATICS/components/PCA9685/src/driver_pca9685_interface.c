/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_pca9685_interface_template.c
 * @brief     driver pca9685 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-02-20
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/02/20  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_pca9685_interface.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#define PCA9685_I2C_MASTER_FREQ_HZ          (400000)        /**< I2C master clock frequency */

static const char *TAG = "pca9685_interface";

extern i2c_master_bus_handle_t i2c_bus_handle;

static i2c_master_dev_handle_t pca9685_handle = NULL;
static uint8_t current_i2c_addr = 0;

static i2c_master_dev_handle_t get_device_handle(uint8_t addr8)
{
    uint8_t addr7 = addr8 >> 1;
    if (pca9685_handle != NULL) {
        if (current_i2c_addr == addr7) {
            return pca9685_handle;
        } else {
            i2c_master_bus_rm_device(pca9685_handle);
            pca9685_handle = NULL;
        }
    }
    
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus_handle is NULL!");
        return NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr7,
        .scl_speed_hz = PCA9685_I2C_MASTER_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &pca9685_handle) != ESP_OK) {
        return NULL;
    }
    current_i2c_addr = addr7;
    return pca9685_handle;
}

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t pca9685_interface_iic_init(void)
{
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus_handle is NULL! Make sure to init I2C master first.");
        return 1;
    }
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t pca9685_interface_iic_deinit(void)
{
    if (pca9685_handle != NULL) {
        i2c_master_bus_rm_device(pca9685_handle);
        pca9685_handle = NULL;
    }
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t pca9685_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    i2c_master_dev_handle_t handle = get_device_handle(addr);
    if (handle == NULL) return 1;

    uint8_t *write_buf = (uint8_t *)malloc(len + 1);
    if (!write_buf) return 1;
    
    write_buf[0] = reg;
    if (len > 0) {
        memcpy(&write_buf[1], buf, len);
    }
    
    esp_err_t err = i2c_master_transmit(handle, write_buf, len + 1, -1);
    free(write_buf);

    return (err == ESP_OK) ? 0 : 1;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t pca9685_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    i2c_master_dev_handle_t handle = get_device_handle(addr);
    if (handle == NULL) return 1;

    esp_err_t err = i2c_master_transmit_receive(handle, &reg, 1, buf, len, -1);
    
    return (err == ESP_OK) ? 0 : 1;
}
/**
 * @brief  interface oe init
 * @return status code
 *         - 0 success
 *         - 1 clock init failed
 * @note   none
 */
uint8_t pca9685_interface_oe_init(void)
{
    return 0;
}

/**
 * @brief  interface oe deinit
 * @return status code
 *         - 0 success
 *         - 1 clock deinit failed
 * @note   none
 */
uint8_t pca9685_interface_oe_deinit(void)
{
    return 0;
}

/**
 * @brief     interface oe write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t pca9685_interface_oe_write(uint8_t value)
{
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void pca9685_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));

}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void pca9685_interface_debug_print(const char *const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    esp_log_writev(ESP_LOG_INFO, TAG, fmt, args);
    va_end(args);
}
