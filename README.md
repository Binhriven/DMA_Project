STM32F103 DMA ADC Example

Dự án này demo cách sử dụng DMA để đọc giá trị ADC từ chân `PA0` trên STM32F103, sau đó gửi dữ liệu qua UART1.

 Chức năng

 Đọc ADC liên tục với DMA (64 mẫu).
 Tính trung bình giá trị ADC.
 Quy đổi sang điện áp (0–3.3V).
 Gửi kết quả qua UART (115200 baud).

Yêu cầu phần cứng

STM32F103C8T6 (hoặc board BluePill).
Kết nối USB-UART với PA9 (TX).

Cách build
1. Mở project trong Keil uVision5.
2. Build và nạp chương trình xuống STM32.
3. Mở Serial Monitor (115200 baud) để xem dữ liệu.

Kết quả
Serial sẽ hiển thị:
ADC(avg): 1234 (0-4065) | Voltage: 0.995 (0 - 3.3) V
