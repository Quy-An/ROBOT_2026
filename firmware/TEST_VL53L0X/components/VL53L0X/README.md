# VL53L0X component (ESP-IDF v5.4.2, I2C new driver)

Component này đóng gói ST VL53L0X API (bản C) và cung cấp lớp “port” cho ESP-IDF v5.4.2 sử dụng I2C new driver (`driver/i2c_master`).

Mục tiêu:
- Dùng được **đầy đủ** các chức năng của ST API: single/continuous ranging, timing budget, limit checks, tuning, interrupt threshold, calibration (offset/xtalk/ref).
- Tích hợp dễ: bạn tự init I2C bus trong app, sau đó “register” bus+address cho component và gọi ST API.

---

## 1) Cấu trúc thư mục

```
components/vl53l0x/
  CMakeLists.txt
  README.md
  include/
    vl53l0x_espidf.h
  src/
    vl53l0x_espidf.c
    vl53l0x_i2c_platform_espidf.c
    vl53l0x_platform_espidf.c
  st_api/
    core/
      inc/   (copy từ Api/core/inc)
      src/   (copy từ Api/core/src)
    platform/
      inc/   (copy từ Api/platform/inc)
```

### Chức năng từng file (phần ESP-IDF)

- `CMakeLists.txt`
  - Khai báo source để build component, include path tới ST headers.
- `include/vl53l0x_espidf.h`
  - API “glue” cho ESP-IDF:
    - `vl53l0x_espidf_i2c_register(...)`: đăng ký thiết bị I2C (addr 7-bit) vào bus_handle.
    - `vl53l0x_espidf_init_st_device(...)`: init struct `VL53L0X_Dev_t` để dùng ST API.
- `src/vl53l0x_i2c_platform_espidf.c`
  - **Port I2C cấp thấp** theo interface ST `vl53l0x_i2c_platform.h`:
    - `VL53L0X_write_multi/read_multi`, byte/word/dword
    - delay: `VL53L0X_platform_wait_us`, `VL53L0X_wait_ms`
  - Bên dưới dùng `i2c_master_transmit` và `i2c_master_transmit_receive`.
- `src/vl53l0x_platform_espidf.c`
  - **Platform wrapper cấp cao** theo ST `vl53l0x_platform.h`:
    - `VL53L0X_WriteMulti/ReadMulti/WrByte/RdWord/...`
    - `VL53L0X_PollingDelay` (delay 1ms)
- `src/vl53l0x_espidf.c`
  - File “giữ chỗ” cho các helper tiện dụng trong tương lai; hiện tại chủ yếu dùng `vl53l0x_espidf.h`.

### ST API (vendor code)

- `st_api/core/inc` + `st_api/core/src`
  - Toàn bộ logic cảm biến: init, đo, cấu hình, calibration, tuning.
- `st_api/platform/inc`
  - Định nghĩa kiểu và prototype cho lớp platform.

---

## 2) Copy ST API vào component (bắt buộc)

Bạn cần copy nguyên trạng từ gói ST đang có:

- Copy `Api/core/inc/*` → `components/vl53l0x/st_api/core/inc/`
- Copy `Api/core/src/*` → `components/vl53l0x/st_api/core/src/`
- Copy `Api/platform/inc/*` → `components/vl53l0x/st_api/platform/inc/`

Không copy các backend Windows:
- `Api/platform/src/vl53l0x_i2c_win_serial_comms.c`
- `Api/platform/src/vl53l0x_i2c_platform.c`
- `Api/platform/src/vl53l0x_platform.c` (component đã có bản ESP-IDF tương đương)

---

## 3) Cách dùng API (Quick start)

### 3.1. Bạn đã có I2C bus init

Bạn đã có:
- `config.h` khai báo `I2C_MASTER_PORT`, `I2C_MASTER_SCL_IO`, `I2C_MASTER_SDA_IO`...
- `i2c_bus_init()` tạo `i2c_master_bus_handle_t bus_handle;`.

### 3.2. Register VL53L0X device vào I2C bus

**Quan trọng:** Component này không tự tạo bus. App phải gọi register để gắn device handle theo địa chỉ I2C.

Ví dụ:

```c
#include "vl53l0x_espidf.h"
#include "vl53l0x_api.h"   // ST API
#include "esp_log.h"

static const char *TAG = "app";

static VL53L0X_Dev_t vl53_dev;

void app_main(void)
{
    i2c_bus_init(); // bạn đã có

    // 7-bit I2C address của VL53L0X (mặc định 0x29)
    ESP_ERROR_CHECK(vl53l0x_espidf_i2c_register(bus_handle, 0x29, 400000));

    // init struct ST device
    vl53l0x_espidf_init_st_device(&vl53_dev, 0x29, 400);

    VL53L0X_DEV dev = vl53l0x_espidf_as_st_dev(&vl53_dev);

    VL53L0X_Error st;

    st = VL53L0X_DataInit(dev);
    if (st) { ESP_LOGE(TAG, "VL53L0X_DataInit failed: %d", st); return; }

    st = VL53L0X_StaticInit(dev);
    if (st) { ESP_LOGE(TAG, "VL53L0X_StaticInit failed: %d", st); return; }

    // (Khuyến nghị) chạy ref calibration/spad management nếu bạn muốn theo đúng flow ST
    // VL53L0X_PerformRefCalibration(...)
    // VL53L0X_PerformRefSpadManagement(...)

    // Đo single-shot (cơ bản)
    VL53L0X_RangingMeasurementData_t measure;
    st = VL53L0X_PerformSingleRangingMeasurement(dev, &measure);
    if (st == VL53L0X_ERROR_NONE) {
        ESP_LOGI(TAG, "Distance=%u mm, status=%u", measure.RangeMilliMeter, measure.RangeStatus);
    } else {
        ESP_LOGE(TAG, "Ranging failed: %d", st);
    }
}
```

Ghi chú:
- `RangeStatus` khác 0 nghĩa là measurement có thể bị lỗi/ngoài điều kiện (cần tra bảng status trong ST doc).

---

## 4) Quy trình khởi tạo chuẩn (khuyến nghị)

Tuỳ use-case, nhưng flow ST thường là:

1. Init I2C bus (app)
2. `vl53l0x_espidf_i2c_register(bus_handle, 0x29, speed_hz)`
3. Tạo `VL53L0X_Dev_t` và set `I2cDevAddr`
4. `VL53L0X_DataInit(dev)`
5. `VL53L0X_StaticInit(dev)`
6. (Tuỳ chọn nhưng khuyến nghị theo ST):
   - `VL53L0X_PerformRefCalibration(dev, &vhv, &phase)`
   - `VL53L0X_PerformRefSpadManagement(dev, &spad_count, &is_aperture)`
7. Cấu hình mode + timing budget/limit checks nếu cần
8. Start đo (single hoặc continuous)

---

## 5) Cấu hình cảm biến (basic → advanced)

### 5.1. Chọn chế độ đo

- Single-shot: dùng `VL53L0X_PerformSingleRangingMeasurement`.
- Continuous:
  - `VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING)`
  - `VL53L0X_StartMeasurement(dev)`
  - Loop:
    - `VL53L0X_GetMeasurementDataReady(dev, &ready)`
    - `VL53L0X_GetRangingMeasurementData(dev, &data)`
    - `VL53L0X_ClearInterruptMask(dev, 0)`
  - `VL53L0X_StopMeasurement(dev)`

(Tên enum/mask cụ thể phụ thuộc header ST; xem `vl53l0x_def.h`.)

### 5.2. Timing budget / hiệu năng

ST API thường hỗ trợ cấu hình timing budget (microseconds):
- `VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, budget_us)`

Gợi ý:
- Budget lớn → chính xác hơn nhưng chậm hơn.
- Budget nhỏ → nhanh hơn nhưng dễ nhiễu hơn.

### 5.3. Long range / high accuracy / high speed profile

Các profile thường là tổ hợp:
- timing budget
- VCSEL pulse period
- limit checks (sigma, signal rate)

Ví dụ các API hay dùng:
- `VL53L0X_SetLimitCheckEnable(dev, check_id, 1/0)`
- `VL53L0X_SetLimitCheckValue(dev, check_id, value)`
- `VL53L0X_SetVcselPulsePeriod(dev, type, period_pclks)`

Khuyến nghị: tham khảo các ví dụ cấu hình trong gói ST (các example “High Accuracy/High Speed/Long Range”).

### 5.4. Calibration (offset / xtalk)

ST API có các hàm trong `vl53l0x_api_calibration.*` để:
- đo và set offset
- đo và set xtalk compensation

Gợi ý thực tế:
- Offset calibration nên làm khi cơ khí/optics ổn định.
- Xtalk calibration nên làm theo environment/tấm cover thực tế.

---

## 6) Đọc dữ liệu

### 6.1. Single ranging

- Ưu điểm: đơn giản.
- Dùng: `VL53L0X_PerformSingleRangingMeasurement(dev, &measure)`.

### 6.2. Continuous ranging

- Ưu điểm: throughput ổn định, dễ làm task loop.
- Dùng: `StartMeasurement` + poll ready + read data.

---

## 7) Nhiều cảm biến trên cùng bus

- Mỗi cảm biến cần địa chỉ I2C khác nhau.
- Flow thường: dùng chân XSHUT để bật từng cảm biến, đổi address bằng ST API, rồi `vl53l0x_espidf_i2c_register` cho từng address.

Ví dụ flow (pseudo):

1) Giữ tất cả XSHUT = 0 (tắt).
2) Bật sensor #1 (XSHUT1=1), address mặc định 0x29.
3) Register 0x29 → init ST device → đổi address sensor #1 sang 0x30.
4) Unregister 0x29, register 0x30.
5) Lặp lại với sensor #2, #3…

Ví dụ code (rút gọn):

```c
// Bật sensor 1 bằng XSHUT (bạn tự điều khiển GPIO)

ESP_ERROR_CHECK(vl53l0x_espidf_i2c_register(bus_handle, 0x29, 400000));

VL53L0X_Dev_t dev1;
vl53l0x_espidf_init_st_device(&dev1, 0x29, 400);
VL53L0X_DEV st1 = vl53l0x_espidf_as_st_dev(&dev1);

VL53L0X_Error st_err;

st_err = VL53L0X_DataInit(st1);
if (st_err != VL53L0X_ERROR_NONE) {
  // handle error
}

st_err = VL53L0X_StaticInit(st1);
if (st_err != VL53L0X_ERROR_NONE) {
  // handle error
}

// Đổi địa chỉ (ST API có trong vl53l0x_api.h)
st_err = VL53L0X_SetDeviceAddress(st1, 0x30);
if (st_err != VL53L0X_ERROR_NONE) {
  // handle error
}

// Cập nhật component mapping theo địa chỉ mới
ESP_ERROR_CHECK(vl53l0x_espidf_i2c_unregister(0x29));
ESP_ERROR_CHECK(vl53l0x_espidf_i2c_register(bus_handle, 0x30, 400000));

dev1.I2cDevAddr = 0x30;
```

Ghi chú:
- Component dùng **địa chỉ I2C 7-bit** (0x29, 0x30…).
- Khi đổi địa chỉ thành công, bạn phải cập nhật cả: (1) mapping handle trong component, (2) `dev.I2cDevAddr` trong struct ST.

---

## 8) Ghi chú kỹ thuật

- Thread-safety: port có mutex nội bộ cho các giao dịch I2C.
- Power cycle/XSHUT: hàm `VL53L0X_cycle_power()` hiện để no-op; nếu bạn có XSHUT GPIO, bạn có thể tự mở rộng.
- Log ST (`VL53L0X_LOG_ENABLE`): mặc định tắt. Nếu bật, bạn cần thay thế/port phần log cho ESP-IDF (vì bản ST gốc phụ thuộc Windows).

---

## 9) Ví dụ cấu hình nâng cao (tham khảo nhanh)

### 9.1 Timing budget

```c
// đơn vị microseconds
VL53L0X_SetMeasurementTimingBudgetMicroSeconds(st, 33000);
```

### 9.2 Inter-measurement period (continuous)

```c
VL53L0X_SetInterMeasurementPeriodMilliSeconds(st, 50);
```

### 9.3 Limit checks (lọc nhiễu)

```c
VL53L0X_SetLimitCheckEnable(st, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
VL53L0X_SetLimitCheckEnable(st, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);

VL53L0X_SetLimitCheckValue(
    st,
    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
    (FixPoint1616_t)(0.1 * 65536) // 0.1 MCPS
);
```

### 9.4 Continuous loop mẫu (polling)

```c
VL53L0X_RangingMeasurementData_t m;
uint8_t ready = 0;

VL53L0X_SetDeviceMode(st, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
VL53L0X_StartMeasurement(st);

while (1) {
    VL53L0X_GetMeasurementDataReady(st, &ready);
    if (ready) {
        VL53L0X_GetRangingMeasurementData(st, &m);
        // dùng m.RangeMilliMeter và m.RangeStatus
        VL53L0X_ClearInterruptMask(st, 0);
    }
    VL53L0X_PollingDelay(st);
}
```

---

## 10) Troubleshooting

- Nếu `VL53L0X_DataInit/StaticInit` fail:
  - kiểm tra wiring (SDA/SCL/GND/VCC), pull-up I2C, và địa chỉ (mặc định 0x29).
  - đảm bảo bạn đã gọi `vl53l0x_espidf_i2c_register(bus_handle, addr, speed_hz)` trước khi gọi ST API.

- Nếu luôn đọc được nhưng `RangeStatus` không tốt:
  - tăng timing budget.
  - bật limit checks và set threshold phù hợp.
  - kiểm tra cover glass/xtalk, cân nhắc xtalk calibration.

- Nếu bạn gắn nhiều task cùng đọc cảm biến:
  - port đã có mutex cho I2C transaction; tuy nhiên vẫn nên serialize ở tầng app nếu bạn chạy nhiều “stateful operations” (start/stop measurement).

- Timeout I2C:
  - timeout mặc định hiện đặt trong component là 1000ms; bạn có thể chỉnh bằng macro `VL53L0X_I2C_TIMEOUT_MS` (trước khi compile component).
