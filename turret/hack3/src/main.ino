//////////////////////////////////////////////////
              //  LICENSE  //
//////////////////////////////////////////////////
#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (IRTurret Control Code)
  * Copyright (c) 2020-2022 Armin Joachimsmeyer (IRremote Library)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
#pragma endregion LICENSE

//////////////////////////////////////////////////
              //  LIBRARIES  //
//////////////////////////////////////////////////
#pragma region LIBRARIES

#include <Arduino.h>
#include <Servo.h>
#include <IRremote.hpp>
#include <HCSR04.h>

#pragma endregion LIBRARIES

//////////////////////////////////////////////////
               //  IR CODES  //
//////////////////////////////////////////////////
#pragma region IR CODES
/*
** if you want to add other remotes (as long as they're on the same protocol above):
** press the desired button and look for a hex code similar to those below (ex: 0x11)
** then add a new line to #define newCmdName 0x11,
** and add a case to the switch statement like case newCmdName: 
** this will let you add new functions to buttons on other remotes!
** the best remotes to try are cheap LED remotes, some TV remotes, and some garage door openers
*/

//defines the specific command code for each button on the remote
#define left 0xD
#define right 0x1B
#define up 0x1D
#define down 0x15
#define vol_minus 0x7
#define vol_plus 0x6
#define mute 0x1E
#define ok 0x19
#define cmd1 0x3
#define cmd2 0xE
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

#define DECODE_NEC  //defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

#pragma endregion IR CODES

//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
#pragma region PINS AND PARAMS

// Declare at the top with other global variables
#define PASSCODE_LENGTH 4
const char CORRECT_PASSCODE[] = "1221"; // Change this to your desired passcode
char passcode[PASSCODE_LENGTH + 1] = ""; // Buffer for user input
bool passcodeEntered = false;

//this is where we store global variables!
UltraSonicDistanceSensor distanceSensor(7, 8);  // trigger pin 7, echo pin 8

Servo yawServo; //names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo; //names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo; //names the servo responsible for ROLL rotation, spins the barrel to fire darts

int yawServoVal = 90; //initialize variables to store the current value of each servo
int pitchServoVal = 100;
int rollServoVal = 90;

int pitchMoveSpeed = 8; //this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
int yawMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
int yawStopSpeed = 90; //value to stop the yaw motor - keep this at 90
int rollMoveSpeed = 90; //this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
int rollStopSpeed = 90; //value to stop the roll motor - keep this at 90

int yawPrecision = 150; // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
int rollPrecision = 158; // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (160ish) for best results.

int pitchMax = 150; // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
int pitchMin = 33; // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax

// Distance sensor mode settings
// Mode 1: detect when distance > MAX_DISTANCE (object moved away)
// Mode 2: detect when distance < MIN_DISTANCE (object approached)
// Mode 3: distance sensing disabled (default)
#define MAX_DISTANCE 40  // cm - threshold for "far" detection
#define MIN_DISTANCE 100   // cm - threshold for "near" detection
#define FIRE_COOLDOWN 500  // ms - minimum time between distance-triggered fires
int distanceMode = 3;     // current mode (1, 2, or 3)
unsigned long lastDistanceFireTime = 0;  // tracks last fire time for cooldown

void shakeHeadYes(int moves = 3); //function prototypes for shakeHeadYes and No for proper compiling
void shakeHeadNo(int moves = 3);
void fire(int moves = 1);  // function prototype for fire
#pragma endregion PINS AND PARAMS

//////////////////////////////////////////////////
              //  S E T U P  //
//////////////////////////////////////////////////
#pragma region SETUP
void setup() { //this is our setup function - it runs once on start up, and is basically where we get everything "set up"
    Serial.begin(9600); // initializes the Serial communication between the computer and the microcontroller

    yawServo.attach(10); //attach YAW servo to pin 10
    pitchServo.attach(11); //attach PITCH servo to pin 11
    rollServo.attach(12); //attach ROLL servo to pin 12

    // Just to know which program is running on my microcontroller
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(9, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin 9"));

    homeServos(); //set servo motors to home position
}
#pragma endregion SETUP

//////////////////////////////////////////////////
               //  L O O P  //
//////////////////////////////////////////////////
#pragma region LOOP

void loop() {
    

    /*
    * Check if received data is available and if yes, try to decode it.
    */
    if (IrReceiver.decode()) {
        // Skip unknown protocols (often noise from ultrasonic sensor)
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            IrReceiver.resume();
            return;
        }

        IrReceiver.printIRResultShort(&Serial);
        Serial.println();

        IrReceiver.resume(); // Enable receiving of the next value


        /*
        * Finally, check the received data and perform actions according to the received command
        */

        switch(IrReceiver.decodedIRData.command){ //this is where the commands are handled

            case up: //pitch up
              if(passcodeEntered) upMove(1);
              break;
            
            case down: //pitch down
              if(passcodeEntered) downMove(1);
              break;

            case left: //fast counterclockwise rotation
              if(passcodeEntered) leftMove(1);
              break;
            
            case right: //fast clockwise rotation
              if(passcodeEntered) rightMove(1);
              break;
            
            case ok: //firing routine 
              if(passcodeEntered) fire();
              //Serial.println("FIRE");
              break;
              
            case star:
              if(passcodeEntered) fireAll();
              delay(50);
              break;

            case vol_minus:  // Set mode 1: detect far objects
              distanceMode = 1;
              Serial.println("Mode 1: detecting distance > MAX_DISTANCE");
              break;

            case vol_plus:   // Set mode 2: detect near objects
              distanceMode = 2;
              Serial.println("Mode 2: detecting distance < MIN_DISTANCE");
              break;

            case mute:       // Set mode 3: disable distance sensing
              distanceMode = 3;
              Serial.println("Mode 3: distance sensing disabled");
              break;

            case cmd1:
              Serial.println("cmd1 entered");
              if (!passcodeEntered) addPasscodeDigit('1');
              break;
            case cmd2:
              Serial.println("cmd2 entered");
              if (!passcodeEntered) addPasscodeDigit('2');
              break;
            // When passcode is fully entered
            default:
              if (strlen(passcode) == PASSCODE_LENGTH) {
                  checkPasscode();
              }
              break;

        }
    }

    // Distance detection based on current mode
    if (distanceMode == 1 || distanceMode == 2) {
        float distance = getDistance();
        bool shouldFire = false;

        if (distanceMode == 1 && distance > MAX_DISTANCE) {
            Serial.println("Mode 1: far detected. Distance: " + String(distance) + "cm.");
            shouldFire = true;
        } else if (distanceMode == 2 && distance > 0 && distance < MIN_DISTANCE) {
            Serial.println("Mode 2: near detected. Distance: " + String(distance) + "cm.");
            shouldFire = true;
        }

        if (shouldFire && (millis() - lastDistanceFireTime >= FIRE_COOLDOWN)) {
            fire(2);
            lastDistanceFireTime = millis();
        }
    }

    delay(50);
}

#pragma endregion LOOP

//////////////////////////////////////////////////
               // FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region FUNCTIONS

// Add these functions somewhere in your code
void checkPasscode() {
    Serial.println("checkPasscode() called");
    if (strcmp(passcode, CORRECT_PASSCODE) == 0) {
        Serial.println("CORRECT PASSCODE");
        passcodeEntered = true;
        shakeHeadYes();
    } else {
        passcodeEntered = false;
        shakeHeadNo();
        Serial.println("INCORRECT PASSCODE");
    }
    passcode[0] = '\0'; // Reset passcode buffer
}

void addPasscodeDigit(char digit) {
    Serial.println("addPasscodeDigit() called");
    if (!passcodeEntered && strlen(passcode) < PASSCODE_LENGTH) {
        strncat(passcode, &digit, 1);
        Serial.println(passcode);
    }
}

void leftMove(int moves){ // function to move left
    for (int i = 0; i < moves; i++){
        yawServo.write(yawStopSpeed + yawMoveSpeed); // adding the servo speed = 180 (full counterclockwise rotation speed)
        delay(yawPrecision); // stay rotating for a certain number of milliseconds
        yawServo.write(yawStopSpeed); // stop rotating
        delay(5); //delay for smoothness
        Serial.println("LEFT");
  }

}

void rightMove(int moves){ // function to move right
  for (int i = 0; i < moves; i++){
      yawServo.write(yawStopSpeed - yawMoveSpeed); //subtracting the servo speed = 0 (full clockwise rotation speed)
      delay(yawPrecision);
      yawServo.write(yawStopSpeed);
      delay(5);
      Serial.println("RIGHT");
  }
}

void upMove(int moves){ // function to tilt up
  for (int i = 0; i < moves; i++){
        if((pitchServoVal+pitchMoveSpeed) < pitchMax){ //make sure the servo is within rotation limits (less than 150 degrees by default)
        pitchServoVal = pitchServoVal + pitchMoveSpeed;//increment the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("UP");
      }
  }
}

void downMove (int moves){ // function to tilt down
  for (int i = 0; i < moves; i++){
      if((pitchServoVal-pitchMoveSpeed) > pitchMin){//make sure the servo is within rotation limits (greater than 35 degrees by default)
        pitchServoVal = pitchServoVal - pitchMoveSpeed; //decrement the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("DOWN");
      }
  }
}

void fire(int moves = 1) { //function for firing a single dart
  for (int i = 0; i < moves; i++){
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision);//time for approximately 60 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    
  }
  delay(5); //delay for smoothness
  Serial.println("FIRING");
}

void fireAll() { //function to fire all 6 darts at once
    rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
    delay(rollPrecision * 6); //time for 360 degrees of rotation
    rollServo.write(rollStopSpeed);//stop rotating the servo
    delay(5); // delay for smoothness
    Serial.println("FIRING ALL");
}

void homeServos(){ // sends servos to home positions
    yawServo.write(yawStopSpeed); //setup YAW servo to be STOPPED (90)
    delay(20);
    rollServo.write(rollStopSpeed); //setup ROLL servo to be STOPPED (90)
    delay(100);
    pitchServo.write(100); //set PITCH servo to 100 degree position
    delay(100);
    pitchServoVal = 100; // store the pitch servo value
    Serial.println("HOMING");
}

void shakeHeadYes(int moves = 3) { //sets the default number of nods to 3, but you can pass in whatever number of nods you want
      Serial.println("YES");

    if ((pitchMax - pitchServoVal) < 15){
      pitchServoVal = pitchServoVal - 15;
    }else if ((pitchServoVal - pitchMin) < 15){
      pitchServoVal = pitchServoVal + 15;
    }
    pitchServo.write(pitchServoVal);

    int startAngle = pitchServoVal; // Current position of the pitch servo
    int lastAngle = pitchServoVal;
    int nodAngle = startAngle + 15; // Angle for nodding motion

    for (int i = 0; i < moves; i++) { // Repeat nodding motion three times
        // Nod up
        for (int angle = startAngle; angle <= nodAngle; angle++) {
            pitchServo.write(angle);
            delay(7); // Adjust delay for smoother motion
        }
        delay(50); // Pause at nodding position
        // Nod down
        for (int angle = nodAngle; angle >= startAngle; angle--) {
            pitchServo.write(angle);
            delay(7); // Adjust delay for smoother motion
        }
        delay(50); // Pause at starting position
    }
}

void shakeHeadNo(int moves = 3) {
    Serial.println("NO");

    for (int i = 0; i < moves; i++) { // Repeat nodding motion three times
        // rotate right, stop, then rotate left, stop
        yawServo.write(140);
        delay(190); // Adjust delay for smoother motion
        yawServo.write(yawStopSpeed);
        delay(50);
        yawServo.write(40);
        delay(190); // Adjust delay for smoother motion
        yawServo.write(yawStopSpeed);
        delay(50); // Pause at starting position
    }
}

float getDistance() {
    float distance = distanceSensor.measureDistanceCm();
    // Serial.print("Distance: ");
    // Serial.print(distance);
    // Serial.println(" cm");
    return distance;
}

void printDistance(float distance) {
    if (distance >= 0) {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    } else {
        Serial.println("Out of range");
    }
}
#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
               //  END CODE  //
//////////////////////////////////////////////////

   