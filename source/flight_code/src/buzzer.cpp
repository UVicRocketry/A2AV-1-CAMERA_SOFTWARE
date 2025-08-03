
#include "buzzer.h"
#include <Arduino.h>




/**
 * @brief Plays the  buzzer melody with specified durations.
 */
void play_buzzer(void) {

    int melody[] = {BUZZER_MELODY};
    int durations[] = {BUZZER_DURATIONS};

   int size = sizeof(durations) / sizeof(int);

    for (int note = 0; note < size; note++) {
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = (TEMPO * 4) / durations[note];
    tone(BUZZER_PIN, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);

    //stop the tone playing:
    noTone(BUZZER_PIN);
  }
}

void buzzer_init(void){
    pinMode(BUZZER_PIN, OUTPUT);
}