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

// IR remote button codes (NEC protocol)
// Note: Some codes overlap - commented out duplicates to avoid conflicts
#define IR_LEFT      0x0D
#define IR_RIGHT     0x1B
#define IR_UP        0x1D
#define IR_DOWN      0x15
#define IR_VOL_MINUS 0x07
#define IR_VOL_PLUS  0x06
#define IR_MUTE      0x1E
#define IR_OK        0x19
#define IR_CMD1      0x03
#define IR_CMD2      0x0E
#define IR_CMD3      0x47
#define IR_CMD4      0x44
#define IR_CMD5      0x40
#define IR_CMD6      0x43
// #define IR_CMD7   0x07  // Same as IR_VOL_MINUS - commented out
// #define IR_CMD8   0x15  // Same as IR_DOWN - commented out
#define IR_CMD9      0x09
// #define IR_CMD0   0x19  // Same as IR_OK - commented out
#define IR_STAR      0x16
// #define IR_HASHTAG 0x0D // Same as IR_LEFT - commented out

#define DECODE_NEC  //defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

#pragma endregion IR CODES

//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
#pragma region PINS AND PARAMS

// Pin definitions - centralized for easy modification
constexpr uint8_t PIN_YAW_SERVO   = 10;
constexpr uint8_t PIN_PITCH_SERVO = 11;
constexpr uint8_t PIN_ROLL_SERVO  = 12;
constexpr uint8_t PIN_IR_RECEIVER = 9;
constexpr uint8_t PIN_TRIG        = 7;
constexpr uint8_t PIN_ECHO        = 8;

// Passcode settings
#define PASSCODE_LENGTH 4
const char CORRECT_PASSCODE[] = "1221"; // Change this to your desired passcode
char passcode[PASSCODE_LENGTH + 1] = ""; // Buffer for user input
bool passcodeEntered = false;

// Hardware objects
UltraSonicDistanceSensor distanceSensor(PIN_TRIG, PIN_ECHO);
Servo yawServo;   // YAW rotation - 360 spin around the base
Servo pitchServo; // PITCH rotation - up and down tilt
Servo rollServo;  // ROLL rotation - spins the barrel to fire darts

// Servo state (mutable - tracks current position)
int yawServoVal   = 90;
int pitchServoVal = 100;
int rollServoVal  = 90;

// Servo configuration (constants - tune these values)
constexpr int PITCH_MOVE_SPEED = 8;   // angle increment for pitch (3-10)
constexpr int YAW_MOVE_SPEED   = 90;  // speed for yaw rotation (10-90)
constexpr int YAW_STOP_SPEED   = 90;  // neutral position to stop yaw
constexpr int ROLL_MOVE_SPEED  = 90;  // speed for roll/fire motor
constexpr int ROLL_STOP_SPEED  = 90;  // neutral position to stop roll

constexpr int YAW_PRECISION  = 150;   // ms - duration of yaw movement (50-500)
constexpr int ROLL_PRECISION = 158;   // ms - duration for one dart fire (~160)

constexpr int PITCH_MAX = 150;  // max pitch angle (< 180)
constexpr int PITCH_MIN = 33;   // min pitch angle (> 0)

// Distance sensor mode settings
enum DistanceMode {
    MODE_FAR_DETECT  = 1,  // fire when distance > MAX_DISTANCE (object moved away)
    MODE_NEAR_DETECT = 2,  // fire when distance < MIN_DISTANCE (object approached)
    MODE_DISABLED    = 3   // distance sensing disabled (default)
};
constexpr int MAX_DISTANCE  = 40;   // cm - threshold for "far" detection
constexpr int MIN_DISTANCE  = 100;  // cm - threshold for "near" detection
constexpr int FIRE_COOLDOWN = 500;  // ms - minimum time between distance-triggered fires
DistanceMode distanceMode = MODE_DISABLED;
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

    yawServo.attach(PIN_YAW_SERVO);
    pitchServo.attach(PIN_PITCH_SERVO);
    rollServo.attach(PIN_ROLL_SERVO);

    // Just to know which program is running on my microcontroller
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(PIN_IR_RECEIVER, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals at pin "));
    Serial.println(PIN_IR_RECEIVER);

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

            case IR_UP:
              if(passcodeEntered) upMove(1);
              break;

            case IR_DOWN:
              if(passcodeEntered) downMove(1);
              break;

            case IR_LEFT:
              if(passcodeEntered) leftMove(1);
              break;

            case IR_RIGHT:
              if(passcodeEntered) rightMove(1);
              break;

            case IR_OK:
              if(passcodeEntered) fire();
              break;

            case IR_STAR:
              if(passcodeEntered) fireAll();
              delay(50);
              break;

            case IR_VOL_MINUS:
              distanceMode = MODE_FAR_DETECT;
              Serial.println(F("Mode 1: detecting distance > MAX_DISTANCE"));
              break;

            case IR_VOL_PLUS:
              distanceMode = MODE_NEAR_DETECT;
              Serial.println(F("Mode 2: detecting distance < MIN_DISTANCE"));
              break;

            case IR_MUTE:
              distanceMode = MODE_DISABLED;
              Serial.println(F("Mode 3: distance sensing disabled"));
              break;

            case IR_CMD1:
              Serial.println(F("cmd1 entered"));
              if (!passcodeEntered) addPasscodeDigit('1');
              break;

            case IR_CMD2:
              Serial.println(F("cmd2 entered"));
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
    if (distanceMode == MODE_FAR_DETECT || distanceMode == MODE_NEAR_DETECT) {
        float distance = getDistance();
        bool shouldFire = false;

        if (distanceMode == MODE_FAR_DETECT && distance > MAX_DISTANCE) {
            Serial.print(F("Mode 1: far detected. Distance: "));
            Serial.print(distance);
            Serial.println(F("cm."));
            shouldFire = true;
        } else if (distanceMode == MODE_NEAR_DETECT && distance > 0 && distance < MIN_DISTANCE) {
            Serial.print(F("Mode 2: near detected. Distance: "));
            Serial.print(distance);
            Serial.println(F("cm."));
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
        yawServo.write(YAW_STOP_SPEED + YAW_MOVE_SPEED); // adding the servo speed = 180 (full counterclockwise rotation speed)
        delay(YAW_PRECISION); // stay rotating for a certain number of milliseconds
        yawServo.write(YAW_STOP_SPEED); // stop rotating
        delay(5); //delay for smoothness
        Serial.println("LEFT");
  }

}

void rightMove(int moves){ // function to move right
  for (int i = 0; i < moves; i++){
      yawServo.write(YAW_STOP_SPEED - YAW_MOVE_SPEED); //subtracting the servo speed = 0 (full clockwise rotation speed)
      delay(YAW_PRECISION);
      yawServo.write(YAW_STOP_SPEED);
      delay(5);
      Serial.println("RIGHT");
  }
}

void upMove(int moves){ // function to tilt up
  for (int i = 0; i < moves; i++){
        if((pitchServoVal+PITCH_MOVE_SPEED) < PITCH_MAX){ //make sure the servo is within rotation limits (less than 150 degrees by default)
        pitchServoVal = pitchServoVal + PITCH_MOVE_SPEED;//increment the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("UP");
      }
  }
}

void downMove (int moves){ // function to tilt down
  for (int i = 0; i < moves; i++){
      if((pitchServoVal-PITCH_MOVE_SPEED) > PITCH_MIN){//make sure the servo is within rotation limits (greater than 35 degrees by default)
        pitchServoVal = pitchServoVal - PITCH_MOVE_SPEED; //decrement the current angle and update
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("DOWN");
      }
  }
}

void fire(int moves = 1) { //function for firing a single dart
  for (int i = 0; i < moves; i++){
    rollServo.write(ROLL_STOP_SPEED + ROLL_MOVE_SPEED);//start rotating the servo
    delay(ROLL_PRECISION);//time for approximately 60 degrees of rotation
    rollServo.write(ROLL_STOP_SPEED);//stop rotating the servo
    
  }
  delay(5); //delay for smoothness
  Serial.println("FIRING");
}

void fireAll() { //function to fire all 6 darts at once
    rollServo.write(ROLL_STOP_SPEED + ROLL_MOVE_SPEED);//start rotating the servo
    delay(ROLL_PRECISION * 6); //time for 360 degrees of rotation
    rollServo.write(ROLL_STOP_SPEED);//stop rotating the servo
    delay(5); // delay for smoothness
    Serial.println("FIRING ALL");
}

void homeServos(){ // sends servos to home positions
    yawServo.write(YAW_STOP_SPEED); //setup YAW servo to be STOPPED (90)
    delay(20);
    rollServo.write(ROLL_STOP_SPEED); //setup ROLL servo to be STOPPED (90)
    delay(100);
    pitchServo.write(100); //set PITCH servo to 100 degree position
    delay(100);
    pitchServoVal = 100; // store the pitch servo value
    Serial.println("HOMING");
}

void shakeHeadYes(int moves = 3) { //sets the default number of nods to 3, but you can pass in whatever number of nods you want
      Serial.println("YES");

    if ((PITCH_MAX - pitchServoVal) < 15){
      pitchServoVal = pitchServoVal - 15;
    }else if ((pitchServoVal - PITCH_MIN) < 15){
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
        yawServo.write(YAW_STOP_SPEED);
        delay(50);
        yawServo.write(40);
        delay(190); // Adjust delay for smoother motion
        yawServo.write(YAW_STOP_SPEED);
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

   