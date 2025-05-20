//
// Created by Kheph on 2025-05-16.
//

#ifndef ACCELEROMETER_FILTER_H
#define ACCELEROMETER_FILTER_H



class accelerometer_EMA {
private:
  static constexpr int WINDOW_SIZE = 50;
  float sampling_freq;
  float threshold;
  uint32_t index;
  uint32_t count;
  float sum;
  int count;
  float buffer[WINDOW_SIZE];

public:
  accelerometer_EMA(float sampling_freq, float threshold): threshold{threshold},index{0},count{0}, sampling_freq{sampling_freq}, {}

};


#endif //ACCELEROMETER_FILTER_H
