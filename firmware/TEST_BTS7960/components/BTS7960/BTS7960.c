#include "BTS7960.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <stdlib.h>

#define BTS7960_PWM_FREQ_HZ 20000
#define BTS7960_PWM_RES LEDC_TIMER_10_BIT
#define BTS7960_PWM_MAX 1023

static void bts7960_pwm_init(gpio_num_t io, ledc_channel_t channel) {
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = io,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = channel,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);
}

void BTS7960_init(void) {
    // Cấu hình timer PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = BTS7960_PWM_RES,
        .freq_hz         = BTS7960_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Khởi tạo các kênh PWM cho động cơ trái
    bts7960_pwm_init(PWM_RPWM_LEFT_IO, PWM_RPWM_LEFT_CHANNEL);
    bts7960_pwm_init(PWM_LPWM_LEFT_IO, PWM_LPWM_LEFT_CHANNEL);
    // Khởi tạo các kênh PWM cho động cơ phải
    bts7960_pwm_init(PWM_RPWM_RIGHT_IO, PWM_RPWM_RIGHT_CHANNEL);
    bts7960_pwm_init(PWM_LPWM_RIGHT_IO, PWM_LPWM_RIGHT_CHANNEL);
}

static void bts7960_set_motor(int pwm, ledc_channel_t rpwm_ch, ledc_channel_t lpwm_ch) {
    int duty = abs(pwm) * BTS7960_PWM_MAX / 100;
    if (pwm > 0) {
        // Tiến: RPWM duty, LPWM 0
        ledc_set_duty(LEDC_LOW_SPEED_MODE, rpwm_ch, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, rpwm_ch);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, lpwm_ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, lpwm_ch);
    } else if (pwm < 0) {
        // Lùi: LPWM duty, RPWM 0
        ledc_set_duty(LEDC_LOW_SPEED_MODE, rpwm_ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, rpwm_ch);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, lpwm_ch, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, lpwm_ch);
    } else {
        // Dừng
        ledc_set_duty(LEDC_LOW_SPEED_MODE, rpwm_ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, rpwm_ch);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, lpwm_ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, lpwm_ch);
    }
}

void BTS7960_set_left(int pwm) {
    bts7960_set_motor(pwm, PWM_RPWM_LEFT_CHANNEL, PWM_LPWM_LEFT_CHANNEL);
}

void BTS7960_set_right(int pwm) {
    bts7960_set_motor(pwm, PWM_RPWM_RIGHT_CHANNEL, PWM_LPWM_RIGHT_CHANNEL);
}

void BTS7960_stop(void) {
    BTS7960_set_left(0);
    BTS7960_set_right(0);
}
