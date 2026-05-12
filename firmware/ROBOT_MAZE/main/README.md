# Tài Liệu Hướng Dẫn - Khối Điều Khiển Robot (Main Component)

Thư mục `main` chứa mã nguồn cấp cao nhất của hệ thống điều khiển robot. Cấu trúc chương trình được thiết kế theo hướng module hóa, phân tách rõ ràng giữa phần khởi tạo cấu hình phần cứng, tính toán động học (kinematics) và luồng điều khiển thời gian thực bằng FreeRTOS.

Dưới đây là mô tả chi tiết về chức năng của từng file, nhiệm vụ của các hàm và luồng hoạt động chính của chương trình.

---

## 1. Chức năng các file trong thư mục `main`

*   **`main.c`**: Là điểm bắt đầu (entry point) của toàn bộ hệ thống. Nhiệm vụ chính là khởi tạo các ngoại vi giao tiếp (như chuẩn I2C), khởi tạo module động học và bắt đầu các FreeRTOS Task điều khiển. Nó cũng đóng vai trò như một bộ não cấp cao để thiết lập kịch bản chạy hoặc nhận lệnh điều hướng.
*   **`config.h`**: Chứa các macro, thông số cấu hình và định nghĩa phần cứng chung (ví dụ: chân GPIO cho I2C, hằng số tốc độ). Giúp việc thay đổi cấu hình phần cứng được tập trung tại một nơi.
*   **`robot_kinematics.c` / `.h`**: Lớp xử lý điều khiển động cơ và toán học (Động học). Nhiệm vụ của nó là nhận các yêu cầu vận tốc tuyến tính ($v_x$) và vận tốc góc ($w_z$), từ đó tự động tính toán ra tốc độ quay cần thiết cho từng bánh xe (động học nghịch) và chuyển đổi thành tín hiệu PWM để gửi xuống phần cứng (module PCA9685).
*   **`robot_controller.c` / `.h`**: Lớp quản lý luồng điều khiển thời gian thực (Real-time Task Controller). Module này sinh ra một FreeRTOS Task chạy ngầm liên tục, với tần số nhất định (như 20Hz), để đều đặn lấy thông số vận tốc mục tiêu và gửi lệnh điều khiển tới động cơ một cách an toàn.
*   **`CMakeLists.txt`**: Cấu hình quy trình biên dịch của dự án sử dụng CMake cho ESP-IDF. Nó liên kết các file nguồn (mã C) và khai báo các thư viện phụ thuộc (như `esp_driver_i2c`, `PCA9685`,...).

---

## 2. Chi tiết các hàm trong từng file

### `main.c`
*   **`i2c_bus_init()`**: Thiết lập và cấu hình giao tiếp I2C Master trên ESP32. Chuẩn bị đường truyền để chuẩn bị giao tiếp với các IC phần cứng qua I2C (PCA9685).
*   **`app_main()`**: Hàm chạy đầu tiên của ESP32. Tuần tự gọi khởi tạo I2C, khởi tạo kinematics, kích hoạt controller task và chạy một vòng lặp vĩnh cửu mẫu để ra lệnh cho robot.

### `robot_kinematics.c`
*   **`kinematics_init()`**: Tính toán trước những hằng số toán học (như bán kính bánh xe $R$, độ rộng trục $B$) để lưu sẵn, giúp tăng tốc tối đa khi hệ thống đang chạy vòng lặp ngầm. Đồng thời khởi động driver PCA9685.
*   **`inverse_kinematics(robot_velocity_t robot)`**: Động học nghịch. Đổi vận tốc mong muốn của robot ($v_x, w_z$) thành vận tốc xoay (rad/s) của bánh trái và bánh phải.
*   **`forward_kinematics(wheel_velocity_t wheel)`**: Động học thuận. Đổi ngược lại từ tốc độ đang xoay của bánh về vận tốc chuẩn hiện tại của thân robot. (Dùng để lấy odometry nếu có).
*   **`map_speed_to_pwm(float rad_s)`**: Chuyển quy đổi dải tốc độ từ đo lường vật lý (rad/s) sang % chiều dài xung (duty cycle 0-100) của mạch PWM.
*   **`motor_control(float left_speed, float right_speed)`**: Hàm giao tiếp hardware trực tiếp, đẩy mức PWM vào các kênh của bộ điều khiển PCA9685 và quyết định chốt quay tiến hay lùi theo dấu điện áp (+ hoặc -).
*   **`move_robot(float vx, float wz)`**: Hàm bao bọc chuyên dụng: Nhận $v_x, w_z$, gọi `inverse_kinematics()`, sau đó gọi quy đổi sang % PWM và tự động xuất ra `motor_control()`.

### `robot_controller.c`
*   **`robot_controller_task_start()`**: Được gọi khi khởi động, khởi tạo sinh ra FreeRTOS Task với tên `robot_ctrl_task`.
*   **`robot_control_task(void *pvParameters)`**: Hàm xử lý cốt lõi liên tục (ví dụ chạy ở 20Hz - chu kỳ 50ms). Nó khóa (lock) lấy `target_vx` và `target_wz` rồi gọi hàm `move_robot()`. Bảo đảm hệ thống update động cơ liên tục một cách mượt mà không quan tâm trên tầng ứng dụng đang bị ngẽn ở đâu.
*   **`robot_controller_set_velocity(float vx, float wz)`**: Api an toàn (Thread-safe) dùng FreeRTOS Spinlocks/Mutex (Critical section) thay đổi vận tốc mục tiêu cho robot.

---

## 3. Luồng hoạt động của Main Component

Luồng sự kiện (Execution Flow) bắt đầu khi cấp nguồn và chạy RTOS:

1.  **Khởi động (`app_main()`)**: 
    Hệ thống bắt đầu từ mức cơ sở, thực hiện kết xuất thông tin log khởi tạo phần mềm điều khiển.
2.  **Khởi tạo Ngoại Vi (Hardware Init)**: 
    *   Hàm `i2c_bus_init()` được gọi để khởi tạo bộ Master điều khiển tín hiệu I2C. Nếu không có I2C thì chương trình sẽ dừng vì không thể nói chuyện với Driver.
3.  **Khởi tạo Động Học & Driver Motor**: 
    *   Hàm `kinematics_init()` được gọi để bắt tay và kiểm tra mạch PCA9685. Hệ thống hoàn tất việc sẵn sàng nhận lệnh truyền PWM.
4.  **Bật luồng chạy ngầm điều khiển động cơ (Task Spawning)**:
    *   `robot_controller_task_start()` được thực hiện và đẩy logic điều khiển vận tốc cơ xuống một luồng ngầm (background task). Kể từ lúc này, hệ thống sở hữu "não tủy" tự lo việc duy trì tốc độ bánh xe thông qua hàm lặp vô tận phía trong, tự ngắt tín hiệu PWM đều đặn.
5.  **Vòng lặp cấp cao điều hướng (Supervisory Main Loop)**:
    *   Quay lại `app_main()`, khối `while(1)` chạy logic cấp não bộ: Quyết định khi nào tiến, khi nào dừng, bằng cách gửi vận tốc mong muốn (`vx`, `wz`) vào hàm `robot_controller_set_velocity()`. Hàm này sẽ cập nhật biến toàn cục một cách an toàn mà không làm gián đoạn luồng PWM phía dưới.
    *   Khi ứng dụng chính cần ngủ chờ (`vTaskDelay`), bên dưới nền luồng `robot_ctrl_task` vẫn tiếp tục bơm chính xác % xung cũ duy trì sự di chuyển mượt mà. Đảm bảo robot hoạt động ổn định và chính xác mà không gặp tình trạng nghẽn thời gian.