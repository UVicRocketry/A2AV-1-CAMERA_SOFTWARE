//
// Created by Kheph on 2025-05-16.
// A class to apply a simple moving average filter to sensor data
// In this case, the filter is being applied to accelerometer sensor data
#include <Arduino.h>
#ifndef ACCELEROMETER_FILTER_H
#define ACCELEROMETER_FILTER_H
template <uint8_t N, class input_t = float, class sum_t = float>
class accelerometer_SMA {

public:
  input_t operator()(input_t input) {
    input /= 2048; // converts raw adc to gs
    // don't need subtract 1 g due to calibration
    sum -= previousInputs[index];
    sum += input;
    previousInputs[index] = input;
    if (++index == N)
      index = 0;
    return (sum) / N;
  }

private:
  uint8_t index{};
  input_t previousInputs[N] = {};
  sum_t sum{};
};

#endif // ACCELEROMETER_FILTER_H