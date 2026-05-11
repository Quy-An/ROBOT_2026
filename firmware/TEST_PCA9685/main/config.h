#ifndef CONFIG_H
#define CONFIG_H

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver_pca9685.h"

/**
 * @brief I2C master configuration
 */
#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_SCL_IO           GPIO_NUM_18
#define I2C_MASTER_SDA_IO           GPIO_NUM_19

/**
 * @brief identify the IOs connected to the peripheral.
 */
#define XSHUT_FRONT_IO                  GPIO_NUM_23     // XSHUT pin of the front VL53L0X
#define INTERUPT_FRONT_IO               GPIO_NUM_22     // Interrupt pin of the front VL53L0X

#define XSHUT_FRONT_LEFT_IO             GPIO_NUM_32     // XSHUT pin of the front left VL53L0X
#define INTERUPT_FRONT_LEFT_IO          GPIO_NUM_33     // Interrupt pin of the front left VL53L0X

#define XSHUT_FRONT_RIGHT_IO            GPIO_NUM_5      // XSHUT pin of the front right VL53L0X
#define INTERUPT_FRONT_RIGHT_IO         GPIO_NUM_17     // Interrupt pin of the front right VL53L0X

#define XSHUT_LEFT_IO                   GPIO_NUM_25     // XSHUT pin of the left VL53L0X
#define INTERUPT_LEFT_IO                GPIO_NUM_26     // Interrupt pin of the left VL53L0X

#define XSHUT_RIGHT_IO                  GPIO_NUM_16     // XSHUT pin of the right VL53L0X
#define INTERUPT_RIGHT_IO               GPIO_NUM_4      // Interrupt pin of the right VL53L0X

#define INTERRUPT_MPU6050_IO            GPIO_NUM_21     // Interrupt pin of the MPU6050

#define TCRT5000_FRONT_ADC_CHANNEL      ADC2_CHANNEL_7  // ADC channel for the front TCRT5000
#define TCRT5000_LEFT_ADC_CHANNEL       ADC2_CHANNEL_6  // ADC channel for the left TCRT5000
#define TCRT5000_RIGHT_ADC_CHANNEL      ADC2_CHANNEL_3  // ADC channel for the right TCRT5000
#define TCRT5000_REAR_ADC_CHANNEL       ADC2_CHANNEL_4  // ADC channel for the rear TCRT5000
#define TCRT5000_DETECT                 ADC2_CHANNEL_2  // ADC channel for the TCRT5000 used for detecting the opponent

#define PWM_RPWM_LEFT                   PCA9685_CHANNEL_0      // RPWM pin of the left motor
#define PWM_LPWM_LEFT                   PCA9685_CHANNEL_1      // LPWM pin of the left motor
#define PWM_RPWM_RIGHT                  PCA9685_CHANNEL_3      // RPWM pin of the right motor
#define PWM_LPWM_RIGHT                  PCA9685_CHANNEL_2      // LPWM pin of the right motor




#endif /* CONFIG_H */