#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include <pitches.h>

// Pin connected to the buzzer
#define BUZZER_PIN 11
#define TEMPO 300 // Beats per minute
// Tetris Melody and Durations
#define BUZZER_MELODY\
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,\
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,\
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,\
  NOTE_C5, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_C5,\
  \
  NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5,\
  NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,\
  NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,\
  NOTE_C5, NOTE_A4, NOTE_A4, REST, \
  \
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,\
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,\
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,\
  NOTE_C5, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_C5,\
  \
  NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5,\
  NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,\
  NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,\
  NOTE_C5, NOTE_A4, NOTE_A4, REST, \
  \
  NOTE_E5, NOTE_C5,\
  NOTE_D5, NOTE_B4,\
  NOTE_C5, NOTE_A4,\
  NOTE_GS4, NOTE_B4, REST,\
  NOTE_E5, NOTE_C5,\
  NOTE_D5, NOTE_B4,\
  NOTE_C5, NOTE_E5, NOTE_A5,\
  NOTE_GS5

#define BUZZER_DURATIONS\
  4, 8, 8, 4, 8, 8,\
  4, 8, 8, 4, 8, 8,\
  4, 8, 4, 4,\
  4, 4, 8, 4, 8, 8,\
  \
  4, 8, 4, 8, 8,\
  4, 8, 4, 8, 8,\
  4, 8, 8, 4, 4,\
  4, 4, 4, 4,\
  \
  4, 8, 8, 4, 8, 8,\
  4, 8, 8, 4, 8, 8,\
  4, 8, 4, 4,\
  4, 4, 8, 4, 8, 8,\
  \
  4, 8, 4, 8, 8,\
  4, 8, 4, 8, 8,\
  4, 8, 8, 4, 4,\
  4, 4, 4, 4,\
  \   
  2, 2,\
  2, 2,\
  2, 2,\
  2, 4, 8,\
  2, 2,\
  2, 2,\
  4, 4, 2,\
  2

void buzzer_init(void);
void play_buzzer(void);



#endif // BUZZER_H