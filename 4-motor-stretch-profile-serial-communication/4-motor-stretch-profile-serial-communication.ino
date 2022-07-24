//#include "Arduino.h"
#include "AccelStepper.h"
#include "elapsedMillis.h"
#include "math.h"

// GLOBAL MOTOR DEFINITIONS
#define MS1 11
#define MS2 12
#define PFD 7
#define SLP 9
#define RST 8

// CAMERA STAGE DEFINITIONS
#define STEPCAMERA 19
#define DIRCAMERA 14
#define SLPCAMERA 2

// GLOBAL MOTOR CONSTANTS
const int MS1Val = HIGH;
const int MS2Val = HIGH;
const int PFDVal = LOW;
const int RSTVal = HIGH;

// GLOBAL MOTOR VARIABLES
int SLPVal = HIGH;

// MOTOR CONSTANTS
const int STEP[4] = {23, 22, 21, 20};
const int DIR[4]  = {18, 17, 16, 15};
const int EN[4]   = { 6,  5,  4,  3};

// MOTOR VARIABLES
float                freqs[4] = { 0.25,  0.25,  0.25,  0.25};
uint32_t            period[4] = {   0 ,    0 ,    0 ,     0};
long                 dists[4] = {  20,   20,   20,   20};
int MotorStartingPositions[4] = { 50,  50,  50,  50};

// MOTOR WAVEFORM VARIABLES
uint32_t sections[5] =  {0,       40,       60,        75,      100};
long locPhase[4][5] = {
                        {0, dists[0], dists[0],        0,         0},
                        {0, dists[1], dists[1],        0,         0},
                        {0, dists[2], dists[2],        0,         0},
                        {0, dists[3], dists[3],        0,         0}
};
float speedPhase[4][5] = {
  {0,        0,        0,        0,         0},
  {0,        0,        0,        0,         0},
  {0,        0,        0,        0,         0},
  {0,        0,        0,        0,         0}
};
uint32_t ts[4][5];

// MOTOR FLAGS
boolean resetFlag[4]      = { LOW,  LOW,  LOW,  LOW};
boolean manualOverride[4] = { LOW,  LOW,  LOW,  LOW};
boolean firstCycle[4]     = {HIGH, HIGH, HIGH, HIGH};
boolean enableState[4]    = { LOW,  LOW,  LOW,  LOW};

// CAMERA VARIABLES
int CameraUnderWell = 1;
int CameraPosition = 10000;
boolean enableStateCamera = LOW; // LOW to disable
boolean ResetCameraFlag = HIGH;

// TIMING VARIABLES
elapsedMicros timers[4];
elapsedMicros timerSerial;

// COMMUNICATION VARIABLES
String SerialInput;
boolean recievedHandshake = LOW;

AccelStepper *stArray[4];
AccelStepper stCamera(1, STEPCAMERA, DIRCAMERA);
int checkPhase(elapsedMicros t, uint32_t ts[]) {
  int inPhase = 4;
  
  if      ( t >= ts[3]                ) { inPhase = 4; } // REST
  else if ((t >= ts[2]) && (t < ts[3])) { inPhase = 3; } // FALL
  else if ((t >= ts[1]) && (t < ts[2])) { inPhase = 2; } // HOLD
  else if ((t >= ts[0]) && (t < ts[1])) { inPhase = 1; } // RISE
  else { inPhase = 4; } // DEFAULT TO REST
  return inPhase;
}

void establishConnection() {

}


void MotorEnable(int MOTOR) {
  enableState[MOTOR] = !enableState[MOTOR];
  if (enableState[MOTOR] == HIGH) { resetFlag[MOTOR] = HIGH; enableState[MOTOR] = LOW;}
  else                            { manualOverride[MOTOR] = LOW;}
}
void MotorDistance(int MOTOR, float d) {
  dists[MOTOR] = d;
  for (int ph = 1; ph < 3; ph++) { locPhase[MOTOR][ph] = dists[MOTOR]; }
  for (int ph = 0; ph < 4; ph++) {
    speedPhase[MOTOR][ph] = (locPhase[MOTOR][ph + 1] - locPhase[MOTOR][ph]) / ( (ts[MOTOR][ph + 1] - ts[MOTOR][ph]) / 1e6 );
    if (isnan(speedPhase[MOTOR][ph])) {
      speedPhase[MOTOR][ph] = 0;
    }
  }
}
void MotorFrequency(int MOTOR, float f) {
  freqs[MOTOR] = f;
  period[MOTOR] = 1e6 / f;
  Serial.println(period[MOTOR]);
  for (int ph = 0; ph < 5; ph++) {
    if   (ph == 4)  { ts[MOTOR][ph] = period[MOTOR]; }
    else            { ts[MOTOR][ph] = sections[ph] * period[MOTOR] / 100; }
  }

  for (int ph = 0; ph < 4; ph++) {
    speedPhase[MOTOR][ph] = (locPhase[MOTOR][ph + 1] - locPhase[MOTOR][ph]) / ( (ts[MOTOR][ph + 1] - ts[MOTOR][ph]) / 1e6 );
    if (isnan(speedPhase[MOTOR][ph])) {
      speedPhase[MOTOR][ph] = 0;
    }
  }
}
void MotorManual(int MOTOR, float m) {
  manualOverride[MOTOR] = HIGH;
  enableState[MOTOR] = LOW;
  stArray[MOTOR]->moveTo(m);
  stArray[MOTOR]->setMaxSpeed(5000);
  if (stArray[MOTOR]->distanceToGo() == 0) {
    enableState[MOTOR] = HIGH;
  }
  
}
void MotorResetPosition(int MOTOR) {
  stArray[MOTOR]->setCurrentPosition(0);
  enableState[MOTOR] = HIGH;
}
void WaveformUpdate(uint32_t RIS, uint32_t HOL, uint32_t FAL) {
  if ( (RIS >= 0) && (HOL > RIS) && (FAL >= HOL) && FAL <= 100 ) {
    sections[1] = RIS; sections[2] = HOL; sections[3] = FAL;
  }

  for (int MOTOR = 0; MOTOR < 4; MOTOR++) {
    enableState[MOTOR] = LOW;
    resetFlag[MOTOR] = HIGH;
    timers[MOTOR] = 0;
    for (int ph = 0; ph < 5; ph++) {
      if   (ph == 4)  { ts[MOTOR][ph] = period[MOTOR]; }
      else            { ts[MOTOR][ph] = sections[ph] * period[MOTOR] / 100; }
    }
    for (int ph = 0; ph < 4; ph++) {
      speedPhase[MOTOR][ph] = (locPhase[MOTOR][ph + 1] - locPhase[MOTOR][ph]) / ( (ts[MOTOR][ph + 1] - ts[MOTOR][ph]) / 1e6 );
      if (isnan(speedPhase[MOTOR][ph])) {
        speedPhase[MOTOR][ph] = 0;
    }
  }
  }
}
void MotorAdjust(int MOTOR, int ADJ) {
  enableState[MOTOR] = LOW;
  manualOverride[MOTOR] = HIGH;
  int cp = stArray[MOTOR]->currentPosition();
  switch (ADJ) {
    case 0: 
      stArray[MOTOR]->moveTo(cp - 10);
      stArray[MOTOR]->setMaxSpeed(5000);
      break;
    case 1:
      stArray[MOTOR]->moveTo(cp + 10);
      stArray[MOTOR]->setMaxSpeed(5000);
      break; 
  }
}
void MotorRetract(int MOTOR) {
  manualOverride[MOTOR] = HIGH;
  enableState[MOTOR] = LOW;
  stArray[MOTOR]->setMaxSpeed(2000);
  stArray[MOTOR]->moveTo(-7000);
}
void CameraMove(int CameraPosition) {
  enableStateCamera = HIGH;
  ResetCameraFlag = LOW;
  digitalWrite(SLPCAMERA, enableStateCamera);
  stCamera.moveTo(CameraPosition); 
}
void CameraReset() {
  Serial.print("RESET IN");
  ResetCameraFlag = HIGH;
  enableStateCamera = HIGH;
  digitalWrite(SLPCAMERA, enableStateCamera);
  stCamera.moveTo(-100000);
//  stCamera.setMaxSpeed(100000);
}
void StartingPositions(int MOTOR, int action) {
  switch (action) {
    case 0: // set starting position
      MotorStartingPositions[MOTOR] = stArray[MOTOR]->currentPosition();
      break;
    case 1: // go to starting position
      enableState[MOTOR] = LOW;
      manualOverride[MOTOR] = HIGH;
      stArray[MOTOR]->moveTo(MotorStartingPositions[MOTOR]);
      break;
    
  }
      
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {
  // put your setup code here, to run once:

  // GLOBAL MOTOR VARIABLE SET-UP
  pinMode(MS1, OUTPUT); digitalWrite(MS1, MS1Val);
  pinMode(MS2, OUTPUT); digitalWrite(MS2, MS2Val);
  pinMode(RST, OUTPUT); digitalWrite(RST, RSTVal);
  pinMode(SLP, OUTPUT); digitalWrite(SLP, SLPVal);
  pinMode(PFD, OUTPUT); digitalWrite(PFD, PFDVal);

  // MOTOR VARIABLE SET-UP
  for (int st = 0; st < 4; st++) {
    pinMode(EN[st], OUTPUT); digitalWrite(EN[st], enableState[st]);

    stArray[st] = new AccelStepper(1, STEP[st], DIR[st]);
    stArray[st]->setMaxSpeed(10000);
    stArray[st]->setAcceleration(100000);

    period[st] = 1e6 / freqs[st];
    for (int ph = 0; ph < 5; ph++) {
      if (ph == 1 || ph == 2) { ts[st][ph] = sections[ph] * period[st] / 100; locPhase[st][ph] = dists[st]; }
      if (ph == 3) {  ts[st][ph] = sections[ph] * period[st] / 100; }
      if (ph == 4) { ts[st][ph] = period[st]; }
    }
    for (int ph = 0; ph < 4; ph++) {
      speedPhase[st][ph] = (locPhase[st][ph + 1] - locPhase[st][ph]) / ( (ts[st][ph + 1] - ts[st][ph]) / 1e6 );
      if (isnan(speedPhase[st][ph])) { speedPhase[st][ph] = 0; }
    }
  }

  // CAMERA STAGE VARIABLE SET-UP
  stCamera.setMaxSpeed(25000);
  stCamera.setAcceleration(100000);
  stCamera.setCurrentPosition(0);
  ResetCameraFlag = HIGH;
  pinMode(SLPCAMERA, OUTPUT); digitalWrite(SLPCAMERA, enableStateCamera);

  // COMMUNICATION SET-UP
  Serial.begin(9600);
  Serial.setTimeout(0);
  establishConnection();
  for (int st = 0; st < 4; st++) {
    enableState[st] = LOW;
    digitalWrite(EN[st], enableState[st]);
    MotorRetract(st);
  }

  int allDistanceToGo = 0;
  for (int st = 0; st < 4; st++ ) {
    allDistanceToGo += stArray[st]->distanceToGo();
  }
  while (allDistanceToGo != 0) {
    allDistanceToGo = 0;
    for (int st = 0; st < 4; st++ ) {
      allDistanceToGo += stArray[st]->distanceToGo();
      stArray[st]->run();
    }
  }

  for (int st = 0; st < 4; st++) {
    stArray[st]->setCurrentPosition(0);
    StartingPositions(st, 1);
  }
  
}

void loop() {
  // COMMUNICATION UPDATE
  if (Serial.available() > 0) {
    SerialInput = Serial.readStringUntil('#');
    Serial.println(SerialInput.substring(0, SerialInput.length() - 1));
    Serial.flush();

    if (SerialInput.substring(0, 1) == "M") {
      int st = SerialInput.substring(1, 2).toInt() - 1;
      MotorEnable(st);
    }
    else if (SerialInput.substring(0, 1) == "D") {
      int st = SerialInput.substring(1, 2).toInt() - 1;
      float d = SerialInput.substring(3, SerialInput.length() - 1).toFloat();
      MotorDistance(st, d);
    }
    else if (SerialInput.substring(0, 1) == "F") {
      int st = SerialInput.substring(1, 2).toInt() - 1;
      float f = SerialInput.substring(3, SerialInput.length() - 1).toFloat();
      MotorFrequency(st, f);
    }
    else if (SerialInput.substring(0, 1) == "O") {
      int st = SerialInput.substring(1, 2).toInt() - 1;
      int m = SerialInput.substring(3, SerialInput.length() - 1).toInt();
      MotorManual(st, m);
    }
    else if (SerialInput.substring(0, 1) == "R") {
      int st = SerialInput.substring(1,2).toInt() - 1;
      MotorResetPosition(st);
    }
    else if (SerialInput.substring(0, 1) == "S") {
      uint32_t RIS = SerialInput.substring(1,3).toInt();
      uint32_t HOL = SerialInput.substring(4,6).toInt();
      uint32_t FAL = SerialInput.substring(7,SerialInput.length() - 1).toInt();
      WaveformUpdate(RIS, HOL, FAL);
    }
    else if (SerialInput.substring(0, 1) == "A") {
      int st = SerialInput.substring(1, 2).toInt() - 1;
      int ADJ = SerialInput.substring(3, SerialInput.length() - 1).toInt();
      MotorAdjust(st, ADJ);
    }
    else if (SerialInput.substring(0, 1) == "C") {
      CameraUnderWell = SerialInput.substring(1,2).toInt() - 1;
      CameraPosition = SerialInput.substring(3,SerialInput.length()-1).toInt();
      CameraMove(CameraPosition);
    }
    else if (SerialInput.substring(0, 1) == "V") {
      CameraReset();
//      Serial.println("RESET");
    }
    else if (SerialInput.substring(0, 1) == "X") {
      int st = SerialInput.substring(1, SerialInput.length() - 1).toInt() - 1;
      MotorRetract(st);
    }
    else if (SerialInput.substring(0, 1) == "P") {
      int st = SerialInput.substring(1, 3).toInt() - 1;
      int action = SerialInput.substring(4, SerialInput.length() - 1).toInt();
      StartingPositions(st, action);
      
    }
  
  
  }


  // MOTOR UPDATE
  for (int st = 0; st < 4; st++) {
    if (timers[st] > period[st])  {
      timers[st] = 0;    // RESET TIMER IF PAST PERIOD LENGTH
    }
    if (firstCycle[st])           {
      timers[st] = 0;  // RESET TIMER IF FIRST CYCLE, SWITCH TO SECOND CYCLE
      firstCycle[st] = LOW;
    }
    if (enableState[st] == HIGH)  {
      timers[st] = 0;  // HOLD AT BEGINNING OF CYCLE IF LOW
    }


    int inPhase = 4;
    inPhase = checkPhase(timers[st], ts[st]);

    if (enableState[st] == LOW) {
      if (resetFlag[st] == HIGH) {
        stArray[st]->setMaxSpeed(2000);
        stArray[st]->moveTo(0);
        stArray[st]->run();
        if (stArray[st]->distanceToGo() == 0) {
          enableState[st] = HIGH;
          resetFlag[st] = LOW;
          timers[st] = 0;
        }
      }
      else if (manualOverride[st] == LOW) {
        stArray[st]->setMaxSpeed(abs(speedPhase[st][inPhase - 1]));
        stArray[st]->moveTo(locPhase[st][inPhase]);
        stArray[st]->run();
      }
      else {
        stArray[st] -> run();
        
      }
    }

  }

  // CAMERA UPDATE
  if (ResetCameraFlag == HIGH) {
    stCamera.run();
    if (stCamera.distanceToGo() == 0) {
      stCamera.setCurrentPosition(0);
      ResetCameraFlag = LOW;
      CameraMove(CameraPosition);
    }
    
  }
  else if (enableStateCamera == HIGH) {
    stCamera.run();
    if (stCamera.distanceToGo() == 0) {
      enableStateCamera = LOW;
      digitalWrite(SLPCAMERA, enableStateCamera);
    }
  }



  
  // TIMER UPDATE
  if (timerSerial > 33 * 1000) {

    for (int st = 0; st < 4; st++) {
      //      Serial.print(timers[st]*100/period[st]);
      //      Serial.print(',');
//      Serial.print(digitalRead(EN[st]));
      Serial.print(stArray[st]->currentPosition());
//            Serial.print(enableState[st]);
      Serial.print(" // ");
    }
    Serial.print(stCamera.currentPosition());
    Serial.print(',');
    Serial.print(ResetCameraFlag);
    Serial.print(',');
    Serial.print(enableStateCamera);
    Serial.print(',');
    Serial.print(',');
    
    Serial.println(' ');
    timerSerial = 0;
  }
}
