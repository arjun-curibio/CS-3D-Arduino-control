#include <Arduino.h>
#include "AccelStepper.h"
#include "elapsedMillis.h"
#include "math.h"
float freqs[4] = {1., 1., 1., 1.};
uint32_t period[4];

uint32_t sections[5] = {0, 40, 60, 75, 100};

// PHASES:      [B  ]      RISE      HOLD      FALL      REST
//     ts:        0   eo_rise   eo_hold   eo_fall   eo_rest [microseconds]
// speedPhase:    0    c_rise    c_hold    c_fall    c_rest [steps/s]
// locationPhase: 0   eo_rise   eo_hold   eo_fall   eo_rest [steps]                        0, maxDistance, maxDistance, 0, 0

// ts calculation: ts[current] = (sections[next] - sections[current]) * period /100        [microseconds]

// speed calculation:                       (locationPhase[next] - locationPhase[current])
//                   speedPhase[current] =  ----------------------------------------------    [steps/microseconds]
//                                                     (ts[next] - ts[current])
//

// HIGH = 1
// LOW = 0

uint32_t ts[4][5];
float speedPhase[4][5] = {
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}
};

boolean recievedHandshake = LOW; uint32_t tHandshake = 0;
elapsedMicros timers[4];
elapsedMicros timerSerial;
boolean inWaiting[4] = {HIGH, HIGH, HIGH, HIGH}; // HIGH to wait
boolean resetFlag[4] = {LOW, LOW, LOW, LOW};
boolean manualMotor[4] = {LOW, LOW, LOW, LOW};

// uint32_t waitTime[4] = {0, 0, 0, 0};

boolean firstCycle[4] = {HIGH, HIGH, HIGH, HIGH};
String val;

boolean motorState[4] = {HIGH, HIGH, HIGH, HIGH}; boolean C_State = LOW;
const int STEP[4] = {23, 22, 21, 20}; const int C_STEP = 19;
const int DIR[4] = {18, 17, 16, 15}; const int C_DIR = 14;
const int EN[4] = {6, 5, 4, 3}; const int C_SLP = 2;
long motorDistances[4] = {200, 200, 200, 200}; int C_Position = 0;
long locationPhase[4][5] = {
  {0, motorDistances[0], motorDistances[0], 0, 0},
  {0, motorDistances[1], motorDistances[1], 0, 0},
  {0, motorDistances[2], motorDistances[2], 0, 0},
  {0, motorDistances[3], motorDistances[3], 0, 0},
};
AccelStepper *stArray[4];
AccelStepper stCamera(1, C_STEP, C_DIR);

const int MS1 = 11; int MSVal1 = HIGH;
const int MS2 = 12; int MSVal2 = HIGH;
const int PFD = 7; int PFDVal = LOW;
const int RST = 8; int RSTVal = HIGH;
const int SLP = 9; int SLPVal = HIGH;

int CameraLocations[4] = {8000, 39000, 79000, 102000};

const int C_EN = 10; int C_ENVal = LOW;
const int C_RST = 1; int C_RSTVal = HIGH;
int checkPhase(elapsedMicros t, uint32_t ts[]) {
  int inPhase = 4;
  if (t >= ts[3]) {
    inPhase = 4; // in REST
  } else if ((t >= ts[2]) && (t < ts[3])) {
    inPhase = 3; // in FALL
  } else if ((t >= ts[1]) && (t < ts[2])) {
    inPhase = 2; // in FALL
  } else if ((t >= ts[0]) && (t < ts[1])) {
    inPhase = 1; // in FALL
  } else {
    inPhase = 4;
  }
  return inPhase;
}


void setup() {
  pinMode(C_RST, OUTPUT); digitalWrite(C_RST, C_RSTVal);
  // pinMode(13, OUTPUT); digitalWrite(13, LOW);
  pinMode(C_EN, OUTPUT); digitalWrite(C_EN, C_ENVal);

  Serial.begin(9600); Serial.setTimeout(0);

  for (int st = 0; st < 4; st++) {
    stArray[st] = new AccelStepper(1, STEP[st], DIR[st]);
    stArray[st]->setMaxSpeed(10000);
    stArray[st]->setAcceleration(100000);

    pinMode(EN[st], OUTPUT); digitalWrite(EN[st], motorState[st]);

    for (int ii = 1; ii < 3; ii++) {
      locationPhase[st][ii] = motorDistances[st];
    }

    period[st] = 1e6 / freqs[st];
    for (int ii = 0; ii < 4; ii++) {
      ts[st][ii] = sections[ii] * period[st] / 100; // microseconds for each phase change
    }
    ts[st][4] = period[st];
    speedPhase[st][0] = (locationPhase[st][1] - locationPhase[st][0]) / ((ts[st][1] - ts[st][0]) / 1e6); // RISE
    speedPhase[st][1] = (locationPhase[st][2] - locationPhase[st][1]) / ((ts[st][2] - ts[st][1]) / 1e6); // HOLD
    speedPhase[st][2] = (locationPhase[st][3] - locationPhase[st][2]) / ((ts[st][3] - ts[st][2]) / 1e6); // FALL
    speedPhase[st][3] = (locationPhase[st][4] - locationPhase[st][3]) / ((ts[st][4] - ts[st][3]) / 1e6); // RELAX

    if (isnan(speedPhase[st][1])) {
      speedPhase[st][1] = 0;
    }


  }

  stCamera.setMaxSpeed(8000);
  stCamera.setAcceleration(100000);
  stCamera.setCurrentPosition(0);
  pinMode(MS1, OUTPUT); digitalWrite(MS1, MSVal1);
  pinMode(MS2, OUTPUT); digitalWrite(MS2, MSVal2);
  pinMode(RST, OUTPUT); digitalWrite(RST, !RSTVal);
  pinMode(PFD, OUTPUT); digitalWrite(PFD, PFDVal);
  pinMode(SLP, OUTPUT); digitalWrite(SLP, !SLPVal);

  pinMode(C_SLP, OUTPUT); digitalWrite(C_SLP, C_State);

}

void loop() {
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // SERIAL COMMANDS FROM PYTHON4
  // TAKEN COMMAND letters:
  //        W: enable/disable
  //        R: reset
  //        F: frequency change
  //        D: distance change
  //        A: adjust motor position
  //        S: waveform change
  //        C: camera movement
  //        X: Emergency motor retract
  //        

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (Serial.available() > 0) {
    val = Serial.readString();
    //Serial.println(val);
    //Serial.println(val);
    if (val.equals("Python Connection Established!\n")) { // establish connection
      //Serial.flush();
      Serial.print("Arduino Connection Established!\n");
      //Serial.flush();
      for (int st = 0; st < 4; st++) {
        motorState[st] = HIGH;
        inWaiting[st] = HIGH;
        resetFlag[st] = LOW;
        // waitTime[st] = 0;
        digitalWrite(EN[st], motorState[st]);
      }
      recievedHandshake = HIGH;
      tHandshake = millis();
    }

    if (val.equals("Python handshake.\n")) {
      Serial.print("Arduino handshake.\n");
      //Serial.flush();
      recievedHandshake = HIGH;
      tHandshake = millis();
    }
    if (val.substring(0, 1) == "C") {
      C_State = HIGH; digitalWrite(C_SLP, C_State);
      Serial.print(val.substring(1, val.length()-1));
      Serial.print(',');
      if (val.substring(1, val.length() - 1) == "R") {
        
        stCamera.setSpeed(30000);
        stCamera.setAcceleration(1000000);

        stCamera.moveTo(-100000); // move camera to starting point
        while (stCamera.distanceToGo() != 0) { stCamera.run();}

        stCamera.setCurrentPosition(0); // reset as 0 point
        // stCamera.moveTo(CameraLocations[0]); // move to under the first motor

        // while (stCamera.distanceToGo() != 0) { stCamera.run();}

      }
      else {
        
        int newCameraLocation = val.substring(1, val.length() - 1).toInt();
        Serial.println(newCameraLocation);
        C_State = HIGH; // enable camera motor
        digitalWrite(C_SLP, C_State);
        stCamera.moveTo(newCameraLocation);
      }
    }
    if (val.substring(0, 1) == "X") {
      for (int st = 0; st < 4; st++) { // enable all motors
        motorState[st] = LOW;
        inWaiting[st] = LOW;
        resetFlag[st] = LOW;
        // waitTime[st] = timers[st];
        digitalWrite(EN[st], motorState[st]);

        stArray[st]->setMaxSpeed(10000); // quickly retract
        stArray[st]->moveTo(-7000); // retract arbitrarily high amount

      }
      int sumDistancesToGo = 0;
      for (int st = 0; st < 4; st++) {
        sumDistancesToGo += stArray[st]->distanceToGo();
      }

      while (sumDistancesToGo != 0) { // retract all motors
        sumDistancesToGo = 0;
        for (int st = 0; st < 4; st++) {
          sumDistancesToGo += stArray[st]->distanceToGo();
          stArray[st]->run();
        }
      }

      for (int st = 0; st < 4; st++) { // enable all motors
        motorState[st] = HIGH;
        inWaiting[st] = HIGH;
        resetFlag[st] = LOW;
        // waitTime[st] = timers[st];
        digitalWrite(EN[st], motorState[st]);
        stArray[st]->setCurrentPosition(0);
      }
    }
    
    if (val.substring(0, 1) == "B") { // boot up program

      for (int st = 0; st < 4; st++) {
        motorState[st] = HIGH;
        inWaiting[st] = HIGH;
        resetFlag[st] = LOW;
        // waitTime[st] = 0;
        digitalWrite(EN[st], HIGH);
        stArray[st]->setCurrentPosition(0);
      }
      digitalWrite(RST, RSTVal);
      digitalWrite(SLP, SLPVal);
      digitalWrite(C_SLP, HIGH);
      Serial.println("Ready");
    } if (val.substring(0, 1) == "Q") { // quit program
      for (int st = 0; st < 4; st++) {
        motorState[st] = HIGH;
        inWaiting[st] = HIGH;
        resetFlag[st] = LOW;
        // waitTime[st] = 0;
        digitalWrite(EN[st], LOW);
      }
      digitalWrite(RST, !RSTVal);
      digitalWrite(SLP, !SLPVal);
    } else if (val.substring(0, 1) == "R") {
      for (int st = 0; st < 4; st++) {
        motorState[st] = LOW;
        inWaiting[st] = HIGH;
        resetFlag[st] = HIGH;
        // waitTime[st] = 0;
        digitalWrite(EN[st], LOW);
      }
    } else if (val.substring(0, 1) == "E") {
      for (int st = 0; st < 4; st++) {
        motorState[st] = !motorState[st];
        inWaiting[st] = LOW;
        resetFlag[st] = LOW;
        // waitTime[st] = timers[st];
        digitalWrite(EN[st], motorState[st]);
      }

    } else if (val.substring(0, 1) == "S") {
      uint32_t num1 = val.substring(2, 4).toInt();
      uint32_t num2 = val.substring(5, 7).toInt();
      uint32_t num3 = val.substring(8, 10).toInt();
      
      if ((num2 > num1) && (num3 >= num2) && (num3 <= 100)) {
        // Serial.println("CHANGED");
        sections[1] = num1;
        sections[2] = num2;
        sections[3] = num3;
      }

      //sections[num1] = uint32_t(val.substring(2, val.length() - 1).toFloat());
      for (int st = 0; st < 4; st++) {
        motorState[st] = LOW;
        inWaiting[st] = HIGH;
        resetFlag[st] = HIGH;
        // waitTime[st] = 0;
        digitalWrite(EN[st], LOW);
        for (int ii = 0; ii < 4; ii++) {
          ts[st][ii] = sections[ii] * period[st] / 100; // microseconds for each phase change
        }
        ts[st][4] = period[st];
        speedPhase[st][0] = (locationPhase[st][1] - locationPhase[st][0]) / ((ts[st][1] - ts[st][0]) / 1e6); // RISE
        speedPhase[st][1] = (locationPhase[st][2] - locationPhase[st][1]) / ((ts[st][2] - ts[st][1]) / 1e6); // HOLD
        speedPhase[st][2] = (locationPhase[st][3] - locationPhase[st][2]) / ((ts[st][3] - ts[st][2]) / 1e6); // FALL
        speedPhase[st][3] = (locationPhase[st][4] - locationPhase[st][3]) / ((ts[st][4] - ts[st][3]) / 1e6); // RELAX
        if (isnan(speedPhase[st][1])) {
          speedPhase[st][1] = 0;
        }
      }
    }



    if (val.substring(0, 1).toInt() > 0) { // individual motor control
      int st = val.substring(0, 1).toInt();
      st = st - 1;
      
      if (val.substring(1, 2) == "S") { // reset motor position to 0
        if (motorState[st] == HIGH) {
          stArray[st]->setCurrentPosition(0);
        }
      }
      if (val.substring(1, 2) == "M") { // manual motor control
        // waitTime[st] = 0;
        inWaiting[st] = HIGH;
        motorState[st] = LOW;
        manualMotor[st] = HIGH;
        digitalWrite(EN[st], motorState[st]);


        int newMotorPosition = val.substring(2, val.length() - 1).toInt();
        // Serial.println(val.substring(2, val.length() - 1));

        stArray[st]->moveTo(newMotorPosition);
        stArray[st]->setMaxSpeed(1000);


      }
      if (val.substring(1, 2) == "W") { // toggle motor state
        if (motorState[st] == LOW) { // if already on, turn off and reset
          motorState[st] = LOW;
          inWaiting[st] = HIGH;
          resetFlag[st] = HIGH;
          // waitTime[st] = 0;
          // motorState will go HIGH in reset loop
        } else { // if already off, turn on
          motorState[st] = LOW;
          inWaiting[st] = LOW;
          // waitTime[st] = 0;
          timers[st] = 0;
        }
        digitalWrite(EN[st], motorState[st]);
        if (manualMotor[st] == HIGH) { // if was recently in manual state, reset
          stArray[st]->setCurrentPosition(0);
          inWaiting[st] = LOW;
          motorState[st] = LOW;
          // waitTime[st] = 0;
          manualMotor[st] = LOW;

          Serial.print("RESET:");
          Serial.println(st);
        }


      } else if (val.substring(1, 2) == "R") { // reset motor

        motorState[st] = LOW;
        manualMotor[st] = LOW;
        inWaiting[st] = HIGH;
        resetFlag[st] = HIGH;
        // waitTime[st] = 0;
        digitalWrite(EN[st], LOW);

      } else if (val.substring(1, 2) == "F") {
        float newFreq = val.substring(2, val.length() - 1).toFloat();
        // Serial.println(val.substring(2, val.length() - 1));

        freqs[st] = newFreq;
        period[st] = 1e6 / freqs[st];
        for (int ii = 0; ii < 4; ii++) {
          ts[st][ii] = sections[ii] * period[st] / 100; // microseconds for each phase change
        }
        ts[st][4] = period[st];
        speedPhase[st][0] = (locationPhase[st][1] - locationPhase[st][0]) / ((ts[st][1] - ts[st][0]) / 1e6); // RISE
        speedPhase[st][1] = (locationPhase[st][2] - locationPhase[st][1]) / ((ts[st][2] - ts[st][1]) / 1e6); // HOLD
        speedPhase[st][2] = (locationPhase[st][3] - locationPhase[st][2]) / ((ts[st][3] - ts[st][2]) / 1e6); // FALL
        speedPhase[st][3] = (locationPhase[st][4] - locationPhase[st][3]) / ((ts[st][4] - ts[st][3]) / 1e6); // RELAX

      } else if (val.substring(1, 2) == "D") { // new motor distance
        float newDistance = val.substring(2, val.length() - 1).toFloat();
        // Serial.println(val.substring(2, val.length() - 1));

        motorDistances[st] = newDistance;
        for (int ii = 1; ii < 3; ii++) {
          locationPhase[st][ii] = motorDistances[st];
        }
        speedPhase[st][0] = (locationPhase[st][1] - locationPhase[st][0]) / ((ts[st][1] - ts[st][0]) / 1e6); // RISE
        speedPhase[st][1] = (locationPhase[st][2] - locationPhase[st][1]) / ((ts[st][2] - ts[st][1]) / 1e6); // HOLD
        speedPhase[st][2] = (locationPhase[st][3] - locationPhase[st][2]) / ((ts[st][3] - ts[st][2]) / 1e6); // FALL
        speedPhase[st][3] = (locationPhase[st][4] - locationPhase[st][3]) / ((ts[st][4] - ts[st][3]) / 1e6); // RELAX

      } else if (val.substring(1, 2) == "A") {
        Serial.println(val);
        digitalWrite(EN[st], LOW);
        digitalWrite(MS1, MSVal1); digitalWrite(MS2, MSVal2);
        if ((inWaiting[st] == HIGH) && (stArray[st]->currentPosition() == 0)) { // only move when in waiting
          int adjustValue = val.substring(2, val.length() - 1).toFloat();
          Serial.println(adjustValue);
          switch (adjustValue) {
            case 0:
              stArray[st]->setCurrentPosition(10);
              break;
            case 1:
              stArray[st]->setCurrentPosition(-10);
              break;
            default:
              break;
          }

          stArray[st]->moveTo(0);
          stArray[st]->setMaxSpeed(1000);
          stArray[st]->runToPosition();
        }
        digitalWrite(EN[st], motorState[st]);
        digitalWrite(MS1, LOW); digitalWrite(MS2, LOW);
      }

    }
  }  // END SERIAL COMMANDS
  
  // LOOP THROUGH EACH STEPPER MOTOR
  for (int st = 0; st < 4; st++) { // loop through each stepper motor    
    if (timers[st] > period[st]) { timers[st] = 0; }

    if (firstCycle[st]) { // if this is the first cycle, need to set t_start to current time
      timers[st] = 0;
      firstCycle[st] = LOW; // set firstCycle tag to LOW
    }

    int inPhase = 4; // re-initialize phase tracker
    inPhase = checkPhase(timers[st], ts[st]);
    if (motorState[st] == LOW) {
      if (resetFlag[st] == HIGH) {
        stArray[st]->setMaxSpeed(2000);
        stArray[st]->moveTo(0);
        stArray[st]->run();
        if (stArray[st]->distanceToGo() == 0) {
          motorState[st] = HIGH; // disable motor
          inWaiting[st] = HIGH; // raise waiting flag
          resetFlag[st] = LOW;    // set reset flag back to 0
          timers[st] = 0; // reset timer
          digitalWrite(EN[st], motorState[st]);
        }
      }
      else if (manualMotor[st] == LOW) {
        stArray[st]->setMaxSpeed(abs(speedPhase[st][inPhase - 1]));
        stArray[st]->moveTo(locationPhase[st][inPhase]);
        stArray[st]->run();
      }
      else {
        stArray[st]->run();

      }
    }

    //Serial.println(' ');
  }

  if (C_State == HIGH) {
    digitalWrite(C_SLP, C_State);
    stCamera.run();
    if (stCamera.distanceToGo() == 0) {
      C_State = LOW;
      digitalWrite(C_SLP, C_State);
    }
  }


  if (timerSerial > 10 * 1000) {

    // for (int st = 0; st < 4; st++) {
    //   Serial.print(motorState[st]);
    //   Serial.print(',');
    //   Serial.print(inWaiting[st]);
    //   Serial.print(',');
    //   Serial.print(manualMotor[st]);
    //   Serial.print(',');
    //   Serial.print(resetFlag[st]);
    //   Serial.print(';');
    // }
    Serial.print("POSITIONS:");
    for (int st = 0; st < 4; st++) {
      
      Serial.print(timers[st]*100/period[st]);
      Serial.print(',');
      Serial.print(stArray[st]->currentPosition());
      Serial.print(";");
    }
    // Serial.print(digitalRead(C_EN));
    // Serial.print(',');
    // Serial.print(digitalRead(C_RST));
    // Serial.print(',');
    // Serial.print(digitalRead(C_SLP));
    // Serial.print(',');
    Serial.println(stCamera.currentPosition());
    
    // Serial.println(' ');
    timerSerial = 0;
  }

  // if (millis() > tHandshake + 10*1000) {
  //   for (int st = 0; st < 4; st++) {
  //     motorState[st] = HIGH; digitalWrite(EN[st], motorState[st]);
  //     C_State = LOW; digitalWrite(C_EN, C_State);

  //   }
  // }
}
