#include <Arduino.h>

constexpr int PIN_UART_K230_RX{5};
constexpr int PIN_UART_K230_TX{6};
constexpr uint32_t k230BaudRate{9600};

HardwareSerial K230Serial(1); // UART1, UART0 reserved for USB debug

void setup() {
  Serial.begin(115200);
  K230Serial.begin(k230BaudRate, SERIAL_8N1, PIN_UART_K230_RX,
                   PIN_UART_K230_TX);
  Serial.println("K230 raw UART echo test");
}

void loop() {
  while (K230Serial.available()) {
    char incomingByte = K230Serial.read();
    Serial.write(incomingByte); // echo raw byte straight through to Serial
  }
}