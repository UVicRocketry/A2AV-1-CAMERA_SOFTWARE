//
// Created by Kheph on 2025-05-16.
//

#ifndef UART_MESSAGES_H
#define UART_MESSAGES_H
#include <HardwareSerial.h>

#define BUFF_SIZE 20
class Camera {
public:
  Camera(Stream &port);
  void ToggleRecording();
  void changeMode();
  void GetDeviceInfo();
  void start_timer();
  void stop_timer();
  unsigned long long get_timer();

  static uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
  static uint8_t calcCrc(uint8_t *buf, uint8_t numBytes);

private:
  Stream &serial_port;
  unsigned long long timer;
  uint8_t txBuf[BUFF_SIZE], rxBuf[BUFF_SIZE];
  
};

#endif // UART_MESSAGES_H
