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


uint32_t ts[4][5];
float speedPhase[4][5] = {
  {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}
};


elapsedMicros timers[4];
elapsedMicros timerSerial;
boolean inWaiting[4] = {HIGH, HIGH, HIGH, HIGH};
boolean resetFlag[4] = {LOW, LOW, LOW, LOW};
uint32_t waitTime[4] = {0, 0, 0, 0};

boolean firstCycle[4] = {HIGH, HIGH, HIGH, HIGH};
String val;

boolean motorState[4] = {HIGH, HIGH, HIGH, HIGH};
const int STEP[4] = {22, 21, 20, 19};
const int DIR[4] = {17, 16, 15, 14};
const int EN[4] = {5, 4, 3, 2};
long motorDistances[4] = {20, 20, 20, 20};
long locationPhase[4][5] = {
  {0, motorDistances[0], motorDistances[0], 0, 0},
  {0, motorDistances[1], motorDistances[1], 0, 0}, 
  {0, motorDistances[2], motorDistances[2], 0, 0}, 
  {0, motorDistances[3], motorDistances[3], 0, 0},
};
AccelStepper *stArray[4];

const int MS1 = 8; int MSVal1 = LOW;
const int MS2 = 9; int MSVal2 = LOW;



void setup() {

  //////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////

  Serial.begin(9600);
  Serial.setTimeout(0);
  for (int st = 0; st < 4; st++) {
    stArray[st] = new AccelStepper(1, STEP[st], DIR[st]);
    stArray[st]->setMaxSpeed(100000);
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

  pinMode(MS1, OUTPUT); digitalWrite(MS1, MSVal1);
  pinMode(MS2, OUTPUT); digitalWrite(MS2, MSVal2);



}

void loop() {
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // SERIAL COMMANDS FROM PYTHON4
  // TAKEN COMMAND letters: W (enable/disable), R (reset), F (frequency change), D (distance change), A (adjust motor position), S (waveform change)

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (Serial.available() > 0) {
    val = Serial.readString();
    if (val.substring(0, 1) == "Q") {
      for (int st = 0; st < 4; st++) {
        motorState[st] = LOW;
        inWaiting[st] = HIGH;
        resetFlag[st] = HIGH;
        waitTime[st] = 0;
        digitalWrite(EN[st], LOW);
      }
    } else if (val.substring(0, 1) == "R") {
      for (int st = 0; st < 4; st++) {
        motorState[st] = LOW;
        inWaiting[st] = HIGH;
        resetFlag[st] = HIGH;
        waitTime[st] = 0;
        digitalWrite(EN[st], LOW);
      }
    } else if (val.substring(0, 1) == "E") {
      for (int st = 0; st < 4; st++) {
        motorState[st] = !motorState[st];
        inWaiting[st] = LOW;
        resetFlag[st] = LOW;
        waitTime[st] = timers[st];
        digitalWrite(EN[st], motorState[st]);
      }

    } else if (val.substring(0, 1) == "S") {
      uint32_t num1 = val.substring(2, 4).toInt();
      uint32_t num2 = val.substring(5, 7).toInt();
      uint32_t num3 = val.substring(8, 10).toInt();
      Serial.print(speedPhase[0][0]);
      Serial.print(',');
      Serial.print(speedPhase[0][1]);
      Serial.print(',');
      Serial.print(speedPhase[0][2]);
      Serial.print(',');
      Serial.print(speedPhase[0][3]);
      Serial.print(',');
      Serial.print(speedPhase[0][4]);
      Serial.print(',');
      Serial.println(' ');
      if ((num2 > num1) && (num3 >= num2) && (num3 <= 100)) {
        Serial.println("CHANGED");
        uint32_t newsections[5] = {0, num1, num2, num3, 100};
        sections[1] = num1;
        sections[2] = num2;
        sections[3] = num3;
      }

      //sections[num1] = uint32_t(val.substring(2, val.length() - 1).toFloat());
      for (int st = 0; st < 4; st++) {
        motorState[st] = LOW;
        inWaiting[st] = HIGH;
        resetFlag[st] = HIGH;
        waitTime[st] = 0;
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
      Serial.print(speedPhase[0][0]);
      Serial.print(',');
      Serial.print(speedPhase[0][1]);
      Serial.print(',');
      Serial.print(speedPhase[0][2]);
      Serial.print(',');
      Serial.print(speedPhase[0][3]);
      Serial.print(',');
      Serial.print(speedPhase[0][4]);
      Serial.println();
    }
    if (val.substring(0, 1).toInt() > 0) {
      int st = val.substring(0, 1).toInt();
      st = st - 1;
      if (val.substring(1, 2) == "W") { ///////////// TOGGLE MOTOR STATE ////////////
        motorState[st] = !motorState[st];
        inWaiting[st] = !inWaiting[st];
        waitTime[st] = timers[st];
        digitalWrite(EN[st], motorState[st]);
        //Serial.println('D');
      } else if (val.substring(1, 2) == "R") {
        motorState[st] = LOW;
        inWaiting[st] = HIGH;
        resetFlag[st] = HIGH;
        waitTime[st] = 0;
        digitalWrite(EN[st], LOW);
        //Serial.println('D');

        //////////////////////////////////////////////////
        // CHANGE FREQUENCY OF ANY MOTOR
        //////////////////////////////////////////////////
      } else if (val.substring(1, 2) == "F") {
        //motorState[st] = LOW;
        //inWaiting[st] = HIGH;
        //resetFlag[st] = HIGH;
        //waitTime[st] = 0;
        //digitalWrite(EN[st], LOW);

        float newFreq = val.substring(2, val.length() - 1).toFloat();
        Serial.println(val.substring(2, val.length() - 1));

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
        //motorState[st] = LOW;
        //inWaiting[st] = HIGH;
        //resetFlag[st] = HIGH;
        //waitTime[st] = 0;
        //digitalWrite(EN[st], LOW);

        float newDistance = val.substring(2, val.length() - 1).toFloat();
        Serial.println(val.substring(2, val.length() - 1));

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
        digitalWrite(MS1, HIGH); digitalWrite(MS2, HIGH);
        if ((inWaiting[st] == HIGH) && (stArray[st]->currentPosition() == 0)) {
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
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // END SERIAL COMMANDS
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // LOOP THROUGH EACH STEPPER MOTOR
  for (int st = 0; st < 4; st++) { // loop through each stepper motor
    if (inWaiting[st]) {
      timers[st] = waitTime[st];
    }
    if (timers[st] > period[st]) {
      timers[st] = 0;
    }

    //    Serial.print(st);
    //    Serial.print(": ");
    //    if (motorState[st]) {
    //      Serial.print("off,");
    //    } else {
    //      Serial.print("on,");
    //    }
    //Serial.print(motorState[st]); // print motor enable state
    //Serial.print(',');

    if (firstCycle[st]) { // if this is the first cycle, need to set t_start to current time
      timers[st] = 0;
      firstCycle[st] = LOW; // set firstCycle tag to LOW
    }

    int inPhase = 4; // re-initialize phase tracker
    //Serial.print(timers[st]);
    //Serial.print(',');
    inPhase = checkPhase(timers[st], ts[st]);

    if (motorState[st] == LOW) {
      if (resetFlag[st] == HIGH) {
        stArray[st]->setMaxSpeed(500);
        stArray[st]->moveTo(0);
        stArray[st]->run();
        if (stArray[st]->distanceToGo() == 0) {
          motorState[st] = HIGH; // disable motor
          inWaiting[st] = HIGH; // reset waiting timer
          resetFlag[st] = LOW;    // set flag back to 0
          timers[st] = 0; // reset timer
          digitalWrite(EN[st], motorState[st]);
        }
      } else {
        stArray[st]->setMaxSpeed(abs(speedPhase[st][inPhase - 1]));
        stArray[st]->moveTo(locationPhase[st][inPhase]);
        stArray[st]->run();
      }
    }

    //Serial.println(' ');
  }

  //  Serial.print(inPhase);
  //  Serial.print(',');
  //  Serial.print(t);
  //  Serial.print(',');

  //delay(100);
  if (timerSerial > 10 * 1000) {
    for (int st = 0; st < 4; st++) {
      Serial.print(stArray[st]->currentPosition());
      Serial.print(",");
    }
    Serial.println(' ');
    timerSerial = 0;
  }

}

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
