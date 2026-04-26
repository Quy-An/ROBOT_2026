#include "vl53l0x_platform.h"  // ST: declares VL53L0X_WriteMulti/ReadMulti...

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef VL53L0X_MAX_I2C_XFER_SIZE
#define VL53L0X_MAX_I2C_XFER_SIZE 64
#endif

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    // Optional: rely on mutex inside i2c_platform implementation.
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) return VL53L0X_ERROR_INVALID_PARAMS;

    uint8_t deviceAddress = Dev->I2cDevAddr;
    int32_t status_int = VL53L0X_write_multi(deviceAddress, index, pdata, (int32_t)count);
    return (status_int == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) return VL53L0X_ERROR_INVALID_PARAMS;

    uint8_t deviceAddress = Dev->I2cDevAddr;
    int32_t status_int = VL53L0X_read_multi(deviceAddress, index, pdata, (int32_t)count);
    return (status_int == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t deviceAddress = Dev->I2cDevAddr;
    return (VL53L0X_write_byte(deviceAddress, index, data) == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t deviceAddress = Dev->I2cDevAddr;
    return (VL53L0X_write_word(deviceAddress, index, data) == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t deviceAddress = Dev->I2cDevAddr;
    return (VL53L0X_write_dword(deviceAddress, index, data) == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t deviceAddress = Dev->I2cDevAddr;
    return (VL53L0X_read_byte(deviceAddress, index, data) == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t deviceAddress = Dev->I2cDevAddr;
    return (VL53L0X_read_word(deviceAddress, index, data) == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t deviceAddress = Dev->I2cDevAddr;
    return (VL53L0X_read_dword(deviceAddress, index, data) == 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;

    uint8_t deviceAddress = Dev->I2cDevAddr;
    uint8_t data = 0;

    if (VL53L0X_read_byte(deviceAddress, index, &data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    data = (data & AndData) | OrData;

    if (VL53L0X_write_byte(deviceAddress, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    // 1ms is enough; VL53L0X APIs typically don't need faster polling.
    vTaskDelay(pdMS_TO_TICKS(1));
    return VL53L0X_ERROR_NONE;
}
