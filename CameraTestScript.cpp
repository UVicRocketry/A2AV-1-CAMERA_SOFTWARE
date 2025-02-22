//Includes: 
#include "CameraFunctions.h"
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//For ADXL375 accelerometer
#include <Adafruit_ADXL375.h>



//Global declaration and definition






//TODO 
//TC1 Manual turn on

//TC2 manual test of turning off


//TC3  test of increasing footage quality (likely have printed text to show changes in quality best) while camera is "rolling"
/*
Goal: Test of footage quality increase with manual input 
Precondition(s): Camera is already on and recording video at a lower quality
Action(s): increase video quality command sent to camera (via function in library)
Expected Behavior: Camera recording quality increases without pause in recording, and continues recording

Expected Behavior: //Button push or some other manual input should cause the filming quality to increase
camera should continue recording with no issues
*/ 
void  manualQualityIncreaseTest(){
    /*
    turn camera on, start recording

    wait for input
    when recieve input, increase the video quality
    call close down funtion that stops recording
    */

}


//TC4 make test of decreasing 
/* 
Goal: Test that camera video quality can be decreased with no issues (currently there is no funtion written for this functionality)
Precondition(s): camera is on and footage is being recorded at a quality that is higher than the lowest possible. 
Action(s): reduce video quality command sent to camera (via a function in the library)
Expected Behavior:  camera footage decreases in quality when it recieves manual input and continues recording with no issues
*/
void  manualQualityDecreaseTest(){

    
}

//TC5 Make test of Stopping footage
/*
Goal: Test that footage can be stopped via commands
Precondition(s): Camera is recording (at any quality) and has sufficient battery life and space remaining for at lease a few minutes more 
Action(s): Send stop recording command to camera via function
Expected Behavior: Recording stops, and captured video is stored correctly and retrievable 
*/

void manualStopRecording(){
    //camera should be recording 
    //wait for button input
    //when button is pushed, call stop recording function
}

//TC6 make test of restarting footage
/*
Goal: Test that camera footage can be restarted after being stopped by a user command
TCs covered:6
Precondition(s): Camera is recording footage
Action(s): Stop recording, (check after that video is saved correctly) restart recording via a command
Expected Behavior: Camera should start recording as usual with no issues
*/



//TC7 Have a very small/very full storage card in camera, and start recording and keep recording 
/*
Goal: Test that the camera can save video correctly when it runs out of space
TCs covered: 7
Precondition(s): Very full or very small SD card is used. Camera is running (at high quality to speed up the test)
Action(s): allow the camera to run until it is full
Expected Behavior: Camera should stop recording, and video should be saved successfully with no issues (be retrievable)
*/

//TC 8 have the batteries run out
/*
Goal: Test that the camera can save video correctly when it loses power
TCs covered: 8
Precondition(s): Camera is recording video as usual
Action(s): pull out batteries, or open the circuit in some other way to simulate losing power (DON'T RUN BATTERIES TO EMPTY- IT WILL DAMAGE THEM)
Expected Behavior: Camera should stop recording, and video shoudl be saved successfully with no issues (be retrievable)
*/

//TC8 have camera/accelerometer experience acceleration above the threshold value, and use that to trigger something in the camera





/*
Use cases: 

as a user, I want to be able to turn on the camera, and have it turn off after a set period of time

As  a user, I want to be able to turn on the camera and start recording right away, increase footage quality after some (non manual) input, then stop after a time limit 

As a user, I want the camera to safely turn off and stop recording when power is lost/batteries go low. 

As a user, I want footage to be successfully saved if the camera runs out of storage

*/


//Template

/*
Goal: 
TCs covered: 
Precondition(s): 
Action(s)
Expected Behavior: 
*/



//Test of basic turn on and turn off using user input 
//Covers TC1 TC2 
//NOTE THIS MIGHT NOT ACTUALLY BE USEFUL OR POSSIBLE
int BasicTurnOnOffTest(){

    return 0; //unsuccessful
}








