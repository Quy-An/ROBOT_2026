#include "vl53l0x_i2c_platform.h"  // ST: declares VL53L0X_write_multi/read_multi...

#include <string.h>
#include <stdlib.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"  // esp_rom_delay_us

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "vl53l0x_i2c";

#ifndef VL53L0X_I2C_TIMEOUT_MS
#define VL53L0X_I2C_TIMEOUT_MS 1000
#endif

typedef struct {
    bool in_use;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint32_t scl_speed_hz;
} vl53l0x_i2c_entry_t;

static vl53l0x_i2c_entry_t s_entries[128];
static SemaphoreHandle_t s_i2c_mutex;

static inline void lock_i2c(void)
{
    if (!s_i2c_mutex) {
        s_i2c_mutex = xSemaphoreCreateMutex();
    }
    if (s_i2c_mutex) {
        (void)xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    }
}

static inline void unlock_i2c(void)
{
    if (s_i2c_mutex) {
        (void)xSemaphoreGive(s_i2c_mutex);
    }
}

static i2c_master_dev_handle_t get_dev(uint8_t addr_7bit)
{
    if (addr_7bit >= 128) return NULL;
    if (!s_entries[addr_7bit].in_use) return NULL;
    return s_entries[addr_7bit].dev;
}

// Public helpers used by app code (declared in vl53l0x_espidf.h)
#include "vl53l0x_espidf.h"

esp_err_t vl53l0x_espidf_i2c_register(i2c_master_bus_handle_t bus,
                                     uint8_t i2c_addr_7bit,
                                     uint32_t scl_speed_hz)
{
    if (!bus) return ESP_ERR_INVALID_ARG;
    if (i2c_addr_7bit >= 128) return ESP_ERR_INVALID_ARG;

    if (s_entries[i2c_addr_7bit].in_use) {
        // Already registered; keep idempotent.
        return ESP_OK;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr_7bit,
        .scl_speed_hz = scl_speed_hz,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device(0x%02x) failed: %s", i2c_addr_7bit, esp_err_to_name(err));
        return err;
    }

    s_entries[i2c_addr_7bit].in_use = true;
    s_entries[i2c_addr_7bit].bus = bus;
    s_entries[i2c_addr_7bit].dev = dev_handle;
    s_entries[i2c_addr_7bit].scl_speed_hz = scl_speed_hz;

    return ESP_OK;
}

esp_err_t vl53l0x_espidf_i2c_unregister(uint8_t i2c_addr_7bit)
{
    if (i2c_addr_7bit >= 128) return ESP_ERR_INVALID_ARG;
    if (!s_entries[i2c_addr_7bit].in_use) return ESP_OK;

    // Remove device from bus
    esp_err_t err = i2c_master_bus_rm_device(s_entries[i2c_addr_7bit].dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_rm_device(0x%02x) failed: %s", i2c_addr_7bit, esp_err_to_name(err));
        // Keep going; still clear entry to avoid leaks across reboot-less tests.
    }

    memset(&s_entries[i2c_addr_7bit], 0, sizeof(s_entries[i2c_addr_7bit]));
    return ESP_OK;
}

// ---- ST platform comms API implementation ----

int32_t VL53L0X_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz)
{
    (void)comms_type;
    (void)comms_speed_khz;
    // App is responsible for calling vl53l0x_espidf_i2c_register().
    return 0;
}

int32_t VL53L0X_comms_close(void)
{
    // No-op: device handles are managed via vl53l0x_espidf_i2c_unregister().
    return 0;
}

int32_t VL53L0X_cycle_power(void)
{
    // Optional: implement if you have an XSHUT GPIO.
    return 0;
}

static int32_t tx(uint8_t addr_7bit, const uint8_t *buf, size_t len)
{
    i2c_master_dev_handle_t dev = get_dev(addr_7bit);
    if (!dev) return 1;

    lock_i2c();
    esp_err_t err = i2c_master_transmit(dev, buf, len, VL53L0X_I2C_TIMEOUT_MS);
    unlock_i2c();

    return (err == ESP_OK) ? 0 : 1;
}

static int32_t txrx(uint8_t addr_7bit, const uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len)
{
    i2c_master_dev_handle_t dev = get_dev(addr_7bit);
    if (!dev) return 1;

    lock_i2c();
    esp_err_t err = i2c_master_transmit_receive(dev, tx_buf, tx_len, rx_buf, rx_len, VL53L0X_I2C_TIMEOUT_MS);
    unlock_i2c();

    return (err == ESP_OK) ? 0 : 1;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if (count < 0) return 1;

    size_t total = (size_t)count + 1;
    uint8_t stack_buf[1 + 64];
    uint8_t *buf = stack_buf;

    if (total > sizeof(stack_buf)) {
        buf = (uint8_t *)malloc(total);
        if (!buf) return 1;
    }

    buf[0] = index;
    if (count > 0 && pdata) {
        memcpy(&buf[1], pdata, (size_t)count);
    }

    int32_t rc = tx(address, buf, total);

    if (buf != stack_buf) free(buf);
    return rc;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if (count < 0 || !pdata) return 1;

    uint8_t reg = index;
    return txrx(address, &reg, 1, pdata, (size_t)count);
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    return VL53L0X_write_multi(address, index, &data, 1);
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF) };
    return VL53L0X_write_multi(address, index, buf, 2);
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    uint8_t buf[4] = {
        (uint8_t)((data >> 24) & 0xFF),
        (uint8_t)((data >> 16) & 0xFF),
        (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)(data & 0xFF)
    };
    return VL53L0X_write_multi(address, index, buf, 4);
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    return VL53L0X_read_multi(address, index, pdata, 1);
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    uint8_t buf[2];
    int32_t rc = VL53L0X_read_multi(address, index, buf, 2);
    if (rc == 0 && pdata) {
        *pdata = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return rc;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    uint8_t buf[4];
    int32_t rc = VL53L0X_read_multi(address, index, buf, 4);
    if (rc == 0 && pdata) {
        *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    }
    return rc;
}

int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    if (wait_us <= 0) return 0;
    esp_rom_delay_us((uint32_t)wait_us);
    return 0;
}

int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    if (wait_ms <= 0) return 0;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
    return 0;
}

int32_t VL53L0X_set_gpio(uint8_t level)
{
    (void)level;
    return 0;
}

int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    if (plevel) *plevel = 0;
    return 0;
}

int32_t VL53L0X_release_gpio(void)
{
    return 0;
}

int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
    if (ptimer_freq_hz) *ptimer_freq_hz = 1000;
    return 0;
}

int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
    if (ptimer_count) *ptimer_count = (int32_t)(xTaskGetTickCount());
    return 0;
}
