# Hướng Dẫn Tái Cấu Trúc Mã Nguồn Firmware BMS Vilas126
Tài liệu này chi tiết hóa cấu trúc tổ chức mới của thư mục `BMS_Vilas126_FW`, giải thích vai trò từng phân lớp (Layers), ánh xạ (Mapping) từ các tệp tin cũ sang tệp tin mới, và hướng dẫn tích hợp mã nguồn mới vào dự án STM32 hiện tại.

---

## 1. Tổng Quan Cấu Trúc Thư Mục Mới
Mã nguồn đã được tổ chức lại theo mô hình phân lớp rõ ràng nhằm nâng cao tính mô-đun (modularity), dễ bảo trì và mở rộng:

```
BMS_Vilas126_FW/
├── Cfg/
│   └── feat_cfg.h        # Cấu hình tính năng phần cứng/phần mềm (Bootloader/App)
├── Drv/
│   ├── can_drv.h         # Khai báo driver giao tiếp CAN Bus
│   ├── can_drv.c         # Triển khai driver giao tiếp CAN Bus (HAL Wrapper)
│   ├── rs485_drv.h       # Khai báo driver điều khiển vi mạch RS485 (UART4/5 DE/RE)
│   └── rs485_drv.c       # Triển khai driver điều khiển vi mạch RS485
├── Proto/
│   ├── can_proto.h       # Khai báo đóng/mở gói giao thức CAN (Pylon/Victron/Growatt)
│   ├── can_proto.c       # Triển khai đóng/mở gói dữ liệu theo từng nhà sản xuất
│   ├── pylon_485.h       # Định nghĩa khung truyền và cấu trúc dữ liệu Pylon RS485
│   └── pylon_485.c       # Mã hóa/Giải mã ASCII Hex và Checksum giao thức Pylon RS485
└── Svc/
    ├── pack.h            # Giao diện dịch vụ quản lý đa khối pin (Multi-pack / Parallel)
    └── pack.c            # Xử lý logic ghép song song, tự động gán địa chỉ, định tuyến
```

---

## 2. Chi Tiết Các Lớp (Layers) và File Ánh Xạ

### 2.1. Lớp Cấu Hình (`Cfg/`)
*   **`feat_cfg.h`**:
    *   *Vai trò*: Quản lý cấu hình biên dịch thông qua macro chọn chế độ hoạt động (như `IS_BOOTLOADER`), giúp tối ưu hóa dung lượng bộ nhớ khi biên dịch cho Bootloader (không bao gồm các driver/giao thức không cần thiết).
    *   *Ánh xạ*: Tách từ các định nghĩa cấu hình hệ thống nằm rải rác trong `main.h`.

### 2.2. Lớp Trình Điều Khiển Thiết Bị (`Drv/`)
*   **`can_drv.h` / `can_drv.c`**:
    *   *Vai trò*: Trừu tượng hóa phần cứng CAN điều khiển bởi bộ thư viện STM32 HAL. Cung cấp các hàm khởi tạo bộ lọc (`Filter_Config`), truyền dữ liệu (`CAN_Drv_Transmit`) và quản lý ngắt nhận dữ liệu.
    *   *Ánh xạ*: Tái cấu trúc từ hàm `CAN_Filter_Config` và logic điều khiển CAN trong `bq_bms_pylon.c` / `main.c`.
*   **`rs485_drv.h` / `rs485_drv.c`**:
    *   *Vai trò*: Quản lý chân điều khiển chiều truyền/nhận (DE/RE) cho hai đường truyền RS485 độc lập (UART4 và UART5), điều chỉnh Baudrate động phục vụ cơ chế Auto-addressing.
    *   *Ánh xạ*: Tách biệt hoàn toàn phần điều khiển chân GPIO (như `RS485_UART4_TX_EN()`, `RS485_UART5_RX_EN()`) từ `bq_bms_485.c`.

### 2.3. Lớp Giao Thức (`Proto/`)
*   **`can_proto.h` / `can_proto.c`**:
    *   *Vai trò*: Chứa mã hóa định dạng dữ liệu cho 4 nhà sản xuất Inverter khác nhau (`Pylontech`, `Victron`, `Growatt`, `Deye`). Chuyển đổi thông số BMS (SOC, SOH, Điện áp, Dòng điện, Lỗi cảnh báo) thành khung tin CAN tương thích.
    *   *Ánh xạ*: Chuyển đổi từ file `bq_bms_pylon.c`.
*   **`pylon_485.h` / `pylon_485.c`**:
    *   *Vai trò*: Mã hóa/giải mã toàn bộ định dạng ASCII Hex của giao thức Pylon RS485. Xử lý tính toán LCHKSUM, CHKSUM và chuyển đổi cấu trúc C sang luồng dữ liệu truyền đi.
    *   *Ánh xạ*: Trích xuất từ các hàm định dạng gói tin (ví dụ `pylon_rs485_analog_pack`, `pylon_rs485_alarm_unpack`) trong `bq_bms_485.c`.

### 2.4. Lớp Dịch Vụ (`Svc/`)
*   **`pack.h` / `pack.c`**:
    *   *Vai trò*: Cung cấp máy trạng thái cho chế độ Tự động gán địa chỉ (`Auto-addressing`), đồng bộ hóa dữ liệu Master-Slave qua RS485 UART5, định tuyến các yêu cầu telemetry từ Inverter/Master xuống tế bào pin.
    *   *Ánh xạ*: Tái cấu trúc máy trạng thái và vòng xử lý byte nhận được (`BQ_Process_UART_Byte`) từ `bq_bms_485.c`.

---

## 3. Hướng Dẫn Tích Hợp Vào Dự Án STM32

Để áp dụng cấu trúc thư mục mới này vào dự án Keil C/STM32CubeIDE hiện tại của bạn, hãy thực hiện theo các bước sau:

### Bước 1: Thêm Đường Dẫn Include Path
Thêm đường dẫn gốc của thư mục mới vào cài đặt compiler của bạn:
*   **Keil uVision**: Vào *Options for Target* -> *C/C++* -> *Include Paths* -> Thêm đường dẫn tới thư mục `BMS_Vilas126_FW`.
*   **STM32CubeIDE**: Vào *Project Properties* -> *C/C++ General* -> *Paths and Symbols* -> *Includes* -> Thêm `BMS_Vilas126_FW`.

### Bước 2: Thay Thế File Cũ Bằng File Mới
1.  **Vô hiệu hóa hoặc xóa các file cũ** khỏi project tree để tránh xung đột định nghĩa trùng lặp:
    *   `bq_bms_485.c` / `bq_bms_485.h`
    *   `bq_bms_pylon.c` / `bq_bms_pylon.h`
2.  **Thêm các file `.c` mới** trong thư mục `BMS_Vilas126_FW` vào Group mã nguồn trong IDE của bạn:
    *   `BMS_Vilas126_FW/Drv/can_drv.c`
    *   `BMS_Vilas126_FW/Drv/rs485_drv.c`
    *   `BMS_Vilas126_FW/Proto/can_proto.c`
    *   `BMS_Vilas126_FW/Proto/pylon_485.c`
    *   `BMS_Vilas126_FW/Svc/pack.c`

### Bước 3: Cập Nhật Điểm Gọi Ngắt (Interrupt Callbacks)
*   **Ngắt Nhận CAN (CAN RX Callback)**:
    Trong `Core/Src/stm32f1xx_it.c` hoặc file xử lý ngắt CAN của bạn, gọi hàm nhận ngắt của driver mới:
    ```c
    #include "Drv/can_drv.h"
    
    void USB_LP_CAN1_RX0_IRQHandler(void) {
        CAN_Drv_IRQHandler(); // Hàm xử lý ngắt nhận CAN mới
    }
    ```
*   **Ngắt Nhận UART (UART RX Callback)**:
    Trong hàm ngắt nhận dữ liệu UART hoặc callback xử lý byte nhận được (thay thế cho hàm xử lý byte cũ):
    ```c
    #include "Svc/pack.h"
    #include "Drv/rs485_drv.h"

    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
        if (huart->Instance == UART4) {
            Pack_Svc_Process_Byte(RS485_PORT_UART4, rx_byte_uart4);
            RS485_Drv_Start_Rx_IT(RS485_PORT_UART4, &rx_byte_uart4);
        } else if (huart->Instance == UART5) {
            Pack_Svc_Process_Byte(RS485_PORT_UART5, rx_byte_uart5);
            RS485_Drv_Start_Rx_IT(RS485_PORT_UART5, &rx_byte_uart5);
        }
    }
    ```

### Bước 4: Cập Nhật Vòng Lặp Chính (Main Loop / Tasks)
Trong vòng lặp `while(1)` chính của chương trình:
*   Gọi hàm cập nhật trạng thái định kỳ:
    ```c
    #include "Svc/pack.h"
    
    // Trong vòng lặp chính
    Pack_Svc_Update();
    ```

---

## 4. Lợi Ích Của Tái Cấu Trúc Này
1.  **Dễ dàng bảo trì**: Khi cần sửa đổi hoặc thêm giao thức CAN mới (ví dụ Sofar, Solis), lập trình viên chỉ cần thao tác trên lớp `Proto/` mà không ảnh hưởng tới driver phần cứng (`Drv/`) hay logic xử lý ứng dụng (`Svc/`).
2.  **Độc lập phần cứng**: Các hàm HAL của STM32 được đóng gói gọn gàng trong thư mục `Drv/`. Khi chuyển đổi vi điều khiển (ví dụ sang ESP32 hoặc GD32), chỉ cần viết lại các tệp tin trong lớp `Drv/`.
3.  **Tối ưu hóa tài nguyên**: Lớp cấu hình `Cfg/feat_cfg.h` giúp bật/tắt linh hoạt các module chức năng thông qua tiền xử lý `#if`, tối đa dung lượng bộ nhớ flash/RAM cho vi điều khiển STM32.
