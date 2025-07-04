//
// Created by Kheph on 2025-05-16.
//
#include "UART_messages.h"
#include <Arduino.h>
#include <HardwareSerial.h>

#include <stdint.h>


Camera::Camera(Stream& port) : serial_port{port}, timer{0}, txBuf{}, rxBuf{} {}

unsigned long long Camera::get_timer() { return timer; }

void Camera::start_timer() { timer = millis(); }

void Camera::stop_timer() { timer = 0; }

uint8_t Camera::crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80)
      crc = (crc << 1) ^ poly;
    else
      crc = crc << 1;
  }
  return crc;
}

uint8_t Camera::calcCrc(uint8_t *buf, uint8_t numBytes) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < numBytes; i++)
    crc = crc8_calc(crc, *(buf + i), 0xd5);
  return crc;
}

void Camera::ToggleRecording() {
  // Send command to turn camera on
  txBuf[0] = 0xCC;
  txBuf[1] = 0x01;
  txBuf[2] = 0x01;
  txBuf[3] = calcCrc(txBuf, 3);
  serial_port.write(txBuf, 4);
}

// Implement functionality to change recording quality
void Camera::changeMode() {
  txBuf[0] = 0xCC;
  txBuf[1] = 0x01;
  txBuf[2] = 0x02;
  txBuf[3] = calcCrc(txBuf, 3);
  serial_port.write(txBuf, 4);
}

void Camera::GetDeviceInfo() {
  // Send command to get device info
  txBuf[0] = 0xCC;
  txBuf[1] = 0x00;
  txBuf[2] = calcCrc(txBuf, 2);
  serial_port.write(txBuf, 3);
}
