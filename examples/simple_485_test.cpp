/**
 * 简单的RS485通信测试程序
 * 用于验证单个RS485通道是否工作正常
 * 
 * 使用方法：
 * 1. 只连接一个编码器到UART0
 * 2. 上传此程序
 * 3. 打开串口监视器查看结果
 */

#include <Arduino.h>

// 配置
#define UART_BAUD_RATE  2500000
#define UART_TX_PIN     1
#define UART_RX_PIN     3
#define UART_DE_PIN     4
#define ENCODER_CMD     0x50
#define RESPONSE_SIZE   22

// RS485控制
void rs485SetTransmit() {
    digitalWrite(UART_DE_PIN, HIGH);
    delayMicroseconds(10);
}

void rs485SetReceive() {
    digitalWrite(UART_DE_PIN, LOW);
    delayMicroseconds(10);
}

void setup() {
    // 初始化调试串口（注意：这会占用UART0）
    // 实际使用时需要使用其他UART进行调试
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n========================================");
    Serial.println("RS485 编码器通信测试");
    Serial.println("========================================\n");
    
    // 配置DE引脚
    pinMode(UART_DE_PIN, OUTPUT);
    rs485SetReceive();
    
    // 警告：由于我们使用Serial进行调试，无法同时用于RS485
    Serial.println("警告：此示例仅用于理解代码逻辑");
    Serial.println("实际测试请使用UART1或UART2，并通过其他方式调试");
    Serial.println("\n如要测试RS485，请：");
    Serial.println("1. 使用UART1(GPIO16/17)或UART2(GPIO25/26)");
    Serial.println("2. 通过LED或其他方式指示状态");
    Serial.println("3. 或使用无线方式（ESP-NOW）输出调试信息\n");
}

void loop() {
    static uint32_t lastTest = 0;
    
    if (millis() - lastTest > 1000) {
        lastTest = millis();
        
        Serial.println("如果这是真实的RS485测试，会执行以下操作：");
        Serial.println("1. 切换到发送模式（DE=HIGH）");
        Serial.println("2. 发送命令字节 0x50");
        Serial.println("3. 切换到接收模式（DE=LOW）");
        Serial.println("4. 等待接收22字节响应");
        Serial.println("5. 显示接收到的数据\n");
        
        // 这里是伪代码示意
        Serial.println("伪代码示例：");
        Serial.println("  rs485SetTransmit();");
        Serial.println("  Serial.write(0x50);");
        Serial.println("  rs485SetReceive();");
        Serial.println("  // 等待并读取22字节...");
        Serial.println();
    }
}

/**
 * 实际的RS485测试代码示例（使用UART1）：
 * 
 * HardwareSerial RS485(1);
 * 
 * void setup() {
 *     Serial.begin(115200);  // 调试用
 *     RS485.begin(2500000, SERIAL_8N1, 16, 17);  // UART1
 *     pinMode(5, OUTPUT);  // DE引脚
 * }
 * 
 * void loop() {
 *     digitalWrite(5, HIGH);  // 发送模式
 *     RS485.write(0x50);
 *     RS485.flush();
 *     
 *     digitalWrite(5, LOW);   // 接收模式
 *     
 *     uint32_t start = millis();
 *     uint8_t buffer[22];
 *     int index = 0;
 *     
 *     while (index < 22 && millis() - start < 10) {
 *         if (RS485.available()) {
 *             buffer[index++] = RS485.read();
 *         }
 *     }
 *     
 *     Serial.printf("接收到 %d 字节: ", index);
 *     for (int i = 0; i < index; i++) {
 *         Serial.printf("%02X ", buffer[i]);
 *     }
 *     Serial.println();
 *     
 *     delay(100);
 * }
 */
