#include "Arduino.h"
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
#define ENCAMERA 13

// Serial definition (for readability)
#define OMV Serial1

// GLOBAL MOTOR CONSTANTS
const int MS1Val = HIGH;
const int MS2Val = HIGH;
const int PFDVal = LOW;
const int RSTVal = HIGH;

// GLOBAL MOTOR VARIABLES
int SLPVal = HIGH;

// MOTOR CONSTANTS
const int STEP[4] = {23, 22, 21, 20}; // STEP PIN
const int DIR[4]  = {18, 17, 16, 15}; // DIR PIN
const int EN[4]   = { 6,  5,  4,  3}; // EN PIN

// MOTOR VARIABLES
float                freqs[4] = { 1,  1,  1,  1};
uint32_t            period[4] = { 0,  0,  0,  0};
long                 dists[4] = {  100,   100,   100,   100}; // [motor steps]
int MotorStartingPositions[4] = { 500,  500,  500,  500};

// MOTOR WAVEFORM VARIABLES
uint32_t sections[5] =  {0,       40,       50,        90,      100};
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
boolean resetFlag[4]          = { LOW,  LOW,  LOW,  LOW}; // MOTOR RESET POSITION FLAG
boolean motorRetractFlag[4]   = { LOW,  LOW,  LOW,  LOW}; // RETRACT FLAG
boolean manualOverride[4]     = { LOW,  LOW,  LOW,  LOW}; // MANUAL TAKEOVER FLAG
boolean firstCycle[4]         = {HIGH, HIGH, HIGH, HIGH}; // WHETHER MOTOR IS IN THE FIRST CYCLE
boolean enableState[4]        = { LOW,  LOW,  LOW,  LOW}; // MOTOR ENABLE/DISABLE FLAG (LOW TO ENABLE)
int motorInitState[4]         = {   0,    0,    0,    0}; // INITIALIZE MOTOR POSITIONS FLAG
boolean MotorInited[4]        = { LOW,  LOW,  LOW,  LOW}; // WHETHER MOTOR IS INITIALIZED
boolean CameraInited[4]       = { LOW,  LOW,  LOW,  LOW}; // WHETHER CAMERA IS INITIALIZED
boolean gotStretchFlag = LOW;
boolean algorithm_magnet_flag = LOW;
boolean foundMax = LOW;
int MotorInitWell = 0;

// CAMERA VARIABLES
int     CameraUnderWell         = 0; // WHICH WELL THE CAMERA IS UNDER
boolean MovingCamera            = LOW; // WHETHER OR NOT THE CAMERA IS MOVING
int     CameraSpeed             = 1250; // SPEED OF CAMERA MOTOR (steps/s)
int     CameraAcceleration      = 1250; // ACCELERATION OF CAMERA MOTOR (steps/s^2)
int     CameraResetPosition     = 13000; // WALL POSITION FOR CAMERA RESET (steps)
int     CameraResetSpeed        = 750; // SPEED OF CAMERA RESET (steps/s)
int     CameraResetAcceleration = 500; // ACCELERATION OF CAMERA RESET (steps/s^2)
int     CameraStartingPosition  = 700; // POSITION AWAY FROM WALL (steps)
int     CameraPosition          = 0; // CURRENT POSITION OF CAMERA (steps)
boolean enableStateCamera       = LOW; // CAMERA MOTOR ENABLE/DISABLE FLAG (LOW TO DISABLE)
boolean ResetCameraFlag         = HIGH; // CAMERA RESET POSITION FLAG
char    rowLabel[4]             = {'D', 'C', 'B', 'A'};

boolean beginMotorHomingFlag    = LOW;
boolean bringDown = LOW;
boolean bringUp = LOW;
// TIMING VARIABLES
elapsedMicros timers[4];
elapsedMicros timerSerial;
elapsedMicros timer_comm;
// COMMUNICATION VARIABLES
String command;
String printstring;
boolean recievedHandshake = LOW;
char serial1buffer[2000];
int sizeOfSerialBuffer;

// OPENMV VARIABLES
float stretchValue = 0.;
float frequencyValue = 0.;
int32_t t_camera = 0;
int last_max_stretch = 0;
int passive_len[4] = {100, 100, 100, 100};
int mag_thresh[4][2]    = {{20, 40}, {20, 40}, {20, 40}, {20, 40}};
int post_thresh[4][2]   = {{0, 15},  {0, 15},  {0, 15},  {0, 15}};
int post_centroid[4][2] = {{0, 0},   {0, 0},   {0, 0},   {0, 0}};

const int LEDPIN = 28;
int LEDVAL = 256; int LEDIN = 100;

boolean HELPERFLAG = LOW;
String HELPERMASK = "None";
int32_t valueArray[50];
boolean OMVStringComplete = LOW;
String mv = "";
String maxs = "[]";
boolean POSTCENTROIDMANUALFLAG  = LOW; // MANUAL OVERRIDE OF POST CENTROID (OPENMV PASSTHROUGH)
boolean recievedMotorInitHandshake = LOW;
boolean waitForCameraInit = LOW;
int k = 0;
AccelStepper *stArray[4];
AccelStepper stCamera(1, STEPCAMERA, DIRCAMERA);

int comm_time = 0;
int checkPhase(elapsedMicros t, uint32_t ts[]) {
  int inPhase = 4;

  if      ( t >= ts[3]                ) {
    inPhase = 4;  // REST
  }
  else if ((t >= ts[2]) && (t < ts[3])) {
    inPhase = 3;  // FALL
  }
  else if ((t >= ts[1]) && (t < ts[2])) {
    inPhase = 2;  // HOLD
  }
  else if ((t >= ts[0]) && (t < ts[1])) {
    inPhase = 1;  // RISE
  }
  else {
    inPhase = 4;  // DEFAULT TO REST
  }
  return inPhase;
}

void establishConnection() {

}
void moveAllMotorsBLOCKING() { // BLOCKING MOTOR MOVEMENT
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
}
void MotorEnable(int MOTOR) { // ENABLE MOTOR
  enableState[MOTOR] = !enableState[MOTOR];
  if      (enableState[MOTOR] == HIGH) {
    resetFlag[MOTOR] = HIGH;
    enableState[MOTOR] = LOW;
  }
  else if (enableState[MOTOR] == LOW)  {
    manualOverride[MOTOR] = LOW;
    timers[MOTOR] = 0;
  }
}
void MotorDistance(int MOTOR, float d) { // CHANGE DISTANCE OF CYCLING
  dists[MOTOR] = d;
  for (int ph = 1; ph < 3; ph++) {
    locPhase[MOTOR][ph] = dists[MOTOR];
  }
  for (int ph = 0; ph < 4; ph++) {
    speedPhase[MOTOR][ph] = (locPhase[MOTOR][ph + 1] - locPhase[MOTOR][ph]) / ( (ts[MOTOR][ph + 1] - ts[MOTOR][ph]) / 1e6 );
    if (isnan(speedPhase[MOTOR][ph])) {
      speedPhase[MOTOR][ph] = 0;
    }
  }
}
void MotorFrequency(int MOTOR, float f) { // CHANGE FREQUENCY OF CYCLING
  freqs[MOTOR] = f;
  period[MOTOR] = 1e6 / f;
  // Serial.println(period[MOTOR]);
  for (int ph = 0; ph < 5; ph++) {
    if   (ph == 4)  {
      ts[MOTOR][ph] = period[MOTOR];
    }
    else            {
      ts[MOTOR][ph] = sections[ph] * period[MOTOR] / 100;
    }
  }

  for (int ph = 0; ph < 4; ph++) {
    speedPhase[MOTOR][ph] = (locPhase[MOTOR][ph + 1] - locPhase[MOTOR][ph]) / ( (ts[MOTOR][ph + 1] - ts[MOTOR][ph]) / 1e6 );
    if (isnan(speedPhase[MOTOR][ph])) {
      speedPhase[MOTOR][ph] = 0;
    }
  }
}
void MotorManual(int MOTOR, int m) { // MANUAL OVERRIDE OF MOTOR POSITION
  manualOverride[MOTOR] = HIGH;
  enableState[MOTOR] = LOW;
  stArray[MOTOR]->moveTo(m);
  stArray[MOTOR]->setMaxSpeed(5000);
  if (stArray[MOTOR]->distanceToGo() == 0) {
    enableState[MOTOR] = HIGH;
  }

}
void MotorResetPosition(int MOTOR) { // RESET MOTOR POSITION
  stArray[MOTOR]->setCurrentPosition(0);
  enableState[MOTOR] = HIGH;
}
void WaveformUpdate(int MOTOR, uint32_t RIS, uint32_t HOL, uint32_t FAL) { // CHANGE WAVEFORM SHAPE
  if ( (RIS >= 0) && (HOL > RIS) && (FAL >= HOL) && FAL <= 100 ) {
    sections[1] = RIS; sections[2] = HOL; sections[3] = FAL;
  }

  // for (int MOTOR = 0; MOTOR < 4; MOTOR++) {
    enableState[MOTOR] = LOW;
    resetFlag[MOTOR] = HIGH;
    timers[MOTOR] = 0;
    for (int ph = 0; ph < 5; ph++) {
      if   (ph == 4)  {
        ts[MOTOR][ph] = period[MOTOR];
      }
      else            {
        ts[MOTOR][ph] = sections[ph] * period[MOTOR] / 100;
      }
    }
    for (int ph = 0; ph < 4; ph++) {
      speedPhase[MOTOR][ph] = (locPhase[MOTOR][ph + 1] - locPhase[MOTOR][ph]) / ( (ts[MOTOR][ph + 1] - ts[MOTOR][ph]) / 1e6 );
      if (isnan(speedPhase[MOTOR][ph])) {
        speedPhase[MOTOR][ph] = 0;
      }
    }
  // }
}
void MotorAdjust(int MOTOR, int ADJ) { // ADJUST POSITION OF MOTOR
  enableState[MOTOR] = LOW;
  manualOverride[MOTOR] = HIGH;
  int cp = stArray[MOTOR]->currentPosition();
  switch (ADJ) {
    case 0:
      stArray[MOTOR]->moveTo(cp - 4);
      stArray[MOTOR]->setMaxSpeed(1000);
      break;
    case 1:
      stArray[MOTOR]->moveTo(cp + 4);
      stArray[MOTOR]->setMaxSpeed(1000);
      break;
  }
}
void MotorRetract(int MOTOR) { // RETRACT MOTOR POSITION TO ZERO
  manualOverride[MOTOR] = HIGH;
  enableState[MOTOR] = LOW;
  motorInitState[MOTOR] = -1; // reset motorInitState flag to -1 (not initialized)
  motorRetractFlag[MOTOR] = HIGH;
  stArray[MOTOR]->setMaxSpeed(5000);
  stArray[MOTOR]->moveTo(-5000);
}
void CameraMove(int CameraPosition) {
  //  enableStateCamera = HIGH;
  //  ResetCameraFlag = LOW;
  //  digitalWrite(SLPCAMERA, enableStateCamera);
  stCamera.moveTo(CameraPosition);
}
void CameraReset() {
  // Serial.print("RESET IN");
  ResetCameraFlag = HIGH;
  //  enableStateCamera = HIGH;
  //  digitalWrite(SLPCAMERA, enableStateCamera);
  stCamera.setAcceleration(CameraResetAcceleration);
  stCamera.setMaxSpeed(CameraResetSpeed);
  CameraMove(stCamera.currentPosition() - CameraResetPosition);
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
void CameraAdjust(int CameraWell, int CameraPosition) {

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
  pinMode(10, OUTPUT);  digitalWrite(10, LOW);
  
  // pinMode(LEDPIN, OUTPUT); analogWrite(LEDPIN, map(LEDIN, 0, 100, 0, 256));
  // LEDVAL = map(LEDIN, 0, 100, 0, 256);

  // OMV.setTX(1);
  // OMV.setRX(0);
  // MOTOR VARIABLE SET-UP
  for (int st = 0; st < 4; st++) {
    pinMode(EN[st], OUTPUT); digitalWrite(EN[st], enableState[st]);

    stArray[st] = new AccelStepper(1, STEP[st], DIR[st]);
    stArray[st]->setMaxSpeed(10000);
    stArray[st]->setAcceleration(100000);
    stArray[st]->setPinsInverted(LOW);

    period[st] = 1e6 / freqs[st];
    for (int ph = 0; ph < 5; ph++) {
      if (ph == 1 || ph == 2) {
        ts[st][ph] = sections[ph] * period[st] / 100;
        locPhase[st][ph] = dists[st];
      }
      if (ph == 3) {
        ts[st][ph] = sections[ph] * period[st] / 100;
      }
      if (ph == 4) {
        ts[st][ph] = period[st];
      }
    }
    for (int ph = 0; ph < 4; ph++) {
      speedPhase[st][ph] = (locPhase[st][ph + 1] - locPhase[st][ph]) / ( (ts[st][ph + 1] - ts[st][ph]) / 1e6 );
      if (isnan(speedPhase[st][ph])) {
        speedPhase[st][ph] = 0;
      }
    }



  }

  // CAMERA STAGE VARIABLE SET-UP
  stCamera.setPinsInverted(HIGH, LOW, LOW);
  stCamera.setMaxSpeed(CameraSpeed);
  stCamera.setAcceleration(CameraAcceleration);
  stCamera.setCurrentPosition(0);
  ResetCameraFlag = LOW;
  pinMode(SLPCAMERA, OUTPUT); digitalWrite(SLPCAMERA, enableStateCamera);
  pinMode (ENCAMERA, OUTPUT); digitalWrite(ENCAMERA, !enableStateCamera);
  // COMMUNICATION SET-UP
  Serial.begin(2000000);
  Serial.setTimeout(0);

  OMV.begin(2000000); // RX: 0, TX: 1

  //   OMV.setTimeout(1);
  sizeOfSerialBuffer = sizeof(serial1buffer);
  OMV.addMemoryForRead(serial1buffer, sizeOfSerialBuffer);
  establishConnection();

  // MOTOR INITIALIZATION
  for (int st = 0; st < 4; st++) {
    MotorRetract(st);
  }

  moveAllMotorsBLOCKING();
  for (int st = 0; st < 4; st++) {
    stArray[st]->setCurrentPosition(0);
    enableState[st] = HIGH;
    manualOverride[st] = LOW;
    resetFlag[st] = HIGH;
  }
  // for (int st = 0; st < 4; st++ ) {
  //   MotorManual(st, 1000);  
  // }
  
  // moveAllMotorsBLOCKING();
  // for (int st = 0; st < 4; st++) {
  //   stArray[st]->setCurrentPosition(0);
  //   enableState[st] = HIGH;
  //   manualOverride[st] = LOW;
  //   resetFlag[st] = HIGH;
  // }
  for (int st = 0; st < 4; st++) {
    motorRetractFlag[st] = LOW;
  }
}

String partialReturn(char del) {
  int index = mv.indexOf('&');
  String val = mv.substring(0, index);
  mv = mv.substring(index + 1);
  return val;
}

void loop() {
  // COMMUNICATION UPDATE
  timer_comm = 0;
  if (Serial.available() > 0) { // FROM COMPUTER
    command = Serial.readString();
    Serial.println(command);
    Serial.flush();
    if (command.substring(0, 8) == "RESETALL") {
      OMV.println("RESET"); // Reset OpenMV
      resetFunc(); // reset Arduino
    }
    if (command.substring(0, 3) == "OMV") { // PASSTHROUGH TO OPENMV 
      OMV.print(command.substring(4, command.length()-1));
    }
    else if (command.substring(0, 3) == "LED") {
      LEDIN = command.substring(4, command.length()-1 ).toInt();
      LEDVAL = map(LEDIN, 0, 100, 0, 256);
      analogWrite(LEDPIN, LEDVAL);
    }
    else if (command.substring(0, 9) == "MOTORINIT") {
      int index = command.indexOf('&');
      command = command.substring(index+1);
      int st = command.substring(0, command.length()-1).toInt();
      // CameraInited[st] = LOW;
      enableState[st] = LOW; // Enable Motor
      manualOverride[st] = LOW; // Manual override disabled
      motorInitState[st] = 1; // Start motor initialization (stage 1)
      MotorInitWell = st;
    }
    else if (command.substring(0, 6) == "UPDATE") {
      OMV.println(command);
      // Serial.print('#');
    }
    else if (command.substring(0, 6) == "CSPEED") {
      CameraSpeed = command.substring(7, command.length() - 1).toInt();
      stCamera.setMaxSpeed(CameraSpeed);
      stCamera.setAcceleration(CameraAcceleration);
    }
    else if (command.substring(0, 6) == "CACCEL") {
      CameraAcceleration = command.substring(7, command.length() - 1).toInt();
      stCamera.setMaxSpeed(CameraSpeed);
      stCamera.setAcceleration(CameraAcceleration);
    }
    // CAMERA COMMANDS PASS THROUGH TO OPENMV
    else if (command.substring(0, 16) == "POSTMANUALTOGGLE") {
      POSTCENTROIDMANUALFLAG = !POSTCENTROIDMANUALFLAG;
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("POSTMANUALTOGGLE");
      OMV.print('&');
      OMV.print(POSTCENTROIDMANUALFLAG);
      OMV.print('&');
      OMV.println(' ');
      // Serial.print('#');


    }
    else if (command.substring(0, 16) == "POSTMANUALENABLE") {
      int del = command.indexOf(',');
      int PostCentroidx = command.substring(16, del).toInt();
      int PostCentroidy = command.substring(del + 1, command.length() - 1).toInt();
      
      POSTCENTROIDMANUALFLAG = HIGH;
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("POSTMANUAL");
      OMV.print('&');
      OMV.print(PostCentroidx);
      OMV.print('&');
      OMV.print(PostCentroidy);
      OMV.print('&');
      OMV.println(' ');
      // Serial.print('#');
    }
    else if (command.substring(0, 10) == "POSTMANUAL") {
      int del = command.indexOf(',');
      int PostCentroidx = command.substring(10, del).toInt();
      int PostCentroidy = command.substring(del + 1, command.length() - 1).toInt();
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("POSTMANUAL");
      OMV.print('&');
      OMV.print(PostCentroidx);
      OMV.print('&');
      OMV.print(PostCentroidy);
      OMV.print('&');
      OMV.println(' ');
      // Serial.print('#');


    }
    else if (command.substring(0, 4) == "INIT") {
      // camera initialization routine
      //CameraUnderWell = command.substring(5,6).toInt();
      if (enableState[CameraUnderWell] == LOW) {
        MotorEnable(CameraUnderWell);
        // MAYBE: wait for motor to fully retract, without pausing other motors
      }
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.println("INIT#");
      // Serial.println('#');
    }
    else if (command.substring(0, 4) == "POST") {
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.println("POST#");
      // Serial.println('#');
    }
    else if (command.substring(0, 6) == "THRESH") {
      // OMV.print(CameraUnderWell);
      // OMV.print('&');
      // OMV.print("recieved");
      // Serial.println('#');
      // OMV.println(CameraUnderWell + "&recieved");
      String input1 = command;
      command = command.substring(6);
      int index = command.indexOf(',');
      int lower = command.substring(0, index).toInt();
      int upper = command.substring(index + 1, command.length() - 1).toInt();
      // String printstring = CameraUnderWell + "&THRESH&" + lower + '&' + upper;
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("THRESH");
      OMV.print('&');
      OMV.print(lower);
      OMV.print('&');
      OMV.println(upper);
      // ('#');
      // OMV.println(printstring);
      // Serial.println(input1+printstring);
    }
    else if (command.substring(0, 12) == "HELPERTOGGLE") {
      HELPERFLAG = !HELPERFLAG;
      OMV.print(CameraUnderWell);
      OMV.print("&HELPERTOGGLE&");
      OMV.print(HELPERFLAG);
      OMV.println('#');

      // Serial.println("Toggled Helper.");

    }
    else if (command.substring(0, 10) == "HELPERMASK") {
      String mask = command.substring(11, command.length() - 1);
      if (HELPERMASK == mask) {
        HELPERMASK = "None";
      }
      else {
        HELPERMASK = mask;
      }

      OMV.print(CameraUnderWell);
      OMV.print("&HELPERMASK&");
      OMV.print(HELPERMASK);
      OMV.println('#');
    }
    else if (command.substring(0, 9) == "PARAMETER") {
      command = command.substring(9);
      // Serial.println(command);

      int i = 0, del = 0;

      for (int idx = 0; idx < 50; idx++) {
        valueArray[idx] = -55; // reset default to -55
      }
      k = 0;
      while (del != -1) {
        // Serial.println("in parsing loop");
        del = command.indexOf(',');
        valueArray[i] = command.substring(0, del).toInt();
        command = command.substring(del + 1);
        k = k + 1;
        i = i + 1;
      }
      // for( int idx = 0; idx < 50; idx++) {
      //   Serial.print(valueArray[idx]);
      //   Serial.print(',');
      // }
      // Serial.println(' ');
      OMV.print(CameraUnderWell);
      OMV.print("&PARAMETER");

      // Serial.print(CameraUnderWell);
      // Serial.print("&PARAMETER");
      for (int idx = 0; idx < k; idx++) {
        OMV.print('&');
        OMV.print(valueArray[idx]);
        // Serial.print('&');
        // Serial.print(valueArray[idx]);

      }
      OMV.println('#');
      // Serial.println('#');

    }
    else if (command.substring(0, 6) == "HELPER") {
      command = command.substring(6);
      // Serial.println(command);

      int i = 0, del = 0;

      for (int idx = 0; idx < 50; idx++) {
        valueArray[idx] = -55; // reset default to -55
      }
      k = 0;
      while (del != -1) {
        // Serial.println("in parsing loop");
        del = command.indexOf(',');
        valueArray[i] = command.substring(0, del).toInt();
        command = command.substring(del + 1);
        k = k + 1;
        i = i + 1;
      }
      // for( int idx = 0; idx < 50; idx++) {
      //   Serial.print(valueArray[idx]);
      //   Serial.print(',');
      // }
      // Serial.println(' ');
      OMV.print(CameraUnderWell);
      OMV.print("&HELPER");

      // Serial.print(CameraUnderWell);
      // Serial.print("&HELPER");
      for (int idx = 0; idx < k; idx++) {
        OMV.print('&');
        OMV.print(valueArray[idx]);
        // Serial.print('&');
        // Serial.print(valueArray[idx]);

      }
      OMV.println('#');
      // Serial.println('#');

    }
    // MOTOR COMMANDS
    else if (command.substring(0, 1) == "M") { // enable/disable Motor
      int st = command.substring(1, 2).toInt();
      MotorEnable(st);
    }
    else if (command.substring(0, 1) == "D") { // change motor Distance
      int st = command.substring(1, 2).toInt();
      float d = command.substring(3, command.length() - 1).toFloat();
      MotorDistance(st, d);
    }
    else if (command.substring(0, 1) == "F") { // change motor Frequency
      int st = command.substring(1, 2).toInt();
      float f = command.substring(3, command.length() - 1).toFloat();
      MotorFrequency(st, f);
    }
    else if (command.substring(0, 1) == "O") { // position Override
      int st = command.substring(1, 2).toInt();
      int m = command.substring(3, command.length() - 1).toInt();
      MotorManual(st, m);
    }
    else if (command.substring(0, 1) == "R") { // Reset
      int st = command.substring(1, 2).toInt();
      MotorResetPosition(st);
    }
    else if (command.substring(0, 1) == "S") { // waveform Shape
      int       st = command.substring(1, 2).toInt();
      uint32_t RIS = command.substring(3, 5).toInt();
      uint32_t HOL = command.substring(6, 8).toInt();
      uint32_t FAL = command.substring(9, command.length() - 1).toInt();
      WaveformUpdate(st, RIS, HOL, FAL);
    }
    else if (command.substring(0, 1) == "A") { // motor Adjustment
      int st = command.substring(1, 2).toInt();
      int ADJ = command.substring(3, command.length() - 1).toInt();
      MotorAdjust(st, ADJ);
    }
    else if (command.substring(0, 1) == "X") { // emergency retract
      int st = command.substring(1, command.length() - 1).toInt();
      MotorRetract(st);
      MotorInited[st] = LOW;
      // MotorDistance(st,25);
    }
    // CAMERA STAGE COMMANDS
    else if (command.substring(0, 1) == "C") { // Camera move
      CameraUnderWell = command.substring(1, 2).toInt();
      CameraPosition = command.substring(3, command.length() - 1).toInt();
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("CHANGE");

      // Serial.println('#');

      CameraMove(CameraPosition);
      // CameraMove(CameraPosition);
    }
    else if (command.substring(0, 1) == "B") { // Camera Speed
      CameraSpeed = command.substring(1, command.length() - 1).toInt();
      stCamera.setMaxSpeed(CameraSpeed);
      stCamera.setAcceleration(CameraAcceleration);

    }
    else if (command.substring(0, 1) == "V") { // camera reset
      CameraReset();
      //      Serial.println("RESET");
    }
    
    else if (command.substring(0, 1) == "P") { // starting Positioning
      int st = command.substring(1, 3).toInt();
      int action = command.substring(4, command.length() - 1).toInt();
      StartingPositions(st, action);
    }
  }


  if (OMV.available() > 0) { // FROM OPENMV
    String mv = OMV.readStringUntil('\n');
    // Serial.println(mv);
    int magic = -22; // initialize to -22 (not used)
    if (mv != "") {
      OMV.flush();
      int index = mv.indexOf('#', 0); // find first #
      magic = mv.substring(0, index).toInt(); // store first substring as magic number
      mv = mv.substring(index + 1); // remove magic number
    }
    if (magic == -47) {
      CameraInited[CameraUnderWell] = LOW;
      MotorInited[CameraUnderWell] = LOW;
      enableState[CameraUnderWell] = HIGH;
    }
    if (magic == -35) { // <t>#<stretch>#<foundMax>#<max_stretch>\n
      gotStretchFlag = HIGH;
      algorithm_magnet_flag = HIGH;
      int index = mv.indexOf('#');
      t_camera = mv.substring(0, index).toFloat();
      mv = mv.substring(index + 1);

      index = mv.indexOf('#');
      stretchValue = mv.substring(0, index).toFloat();
      mv = mv.substring(index + 1);

      index = mv.indexOf('#');
      foundMax = mv.substring(0,index).toInt();
      mv = mv.substring(index + 1);

      index = mv.indexOf('[');
      int index2 = mv.indexOf(']');
      maxs = mv.substring(index, index2+1);
    }
    if (magic == -36) {  // backup magnet <t>#<stretch>#<foundMax>#<max_stretch>\n
      gotStretchFlag = HIGH;
      algorithm_magnet_flag = LOW;
      int index = mv.indexOf('#');
      t_camera = mv.substring(0, index).toFloat();
      mv = mv.substring(index + 1);

      index = mv.indexOf('#');
      stretchValue = mv.substring(0, index).toFloat();
      mv = mv.substring(index + 1);

      index = mv.indexOf('#');
      foundMax = mv.substring(0,index).toInt();
      mv = mv.substring(index + 1);

      index = mv.indexOf('[');
      int index2 = mv.indexOf(']');
      maxs = mv.substring(index, index2+1);
    }
    if (magic == -43) { // <t>&<current_well>&<passive_length>&<magnet_thresh>&<post_threshold>&<centroid_post>\n
      CameraInited[CameraUnderWell] = HIGH;

      int index = mv.indexOf('#');
      t_camera = mv.substring(0, index).toFloat();
      mv = mv.substring(index + 1);

      index = mv.indexOf('#');
      CameraUnderWell = mv.substring(0, index).toInt();
      mv = mv.substring(index + 1);
      
      waitForCameraInit = LOW;
      index = mv.indexOf('#');
      passive_len[CameraUnderWell] = mv.substring(0, index).toInt();
      mv = mv.substring(index + 1); // remove magic number

      index = mv.indexOf('#');
      int index2 = mv.indexOf(',', index);
      mag_thresh[CameraUnderWell][0] = mv.substring(1, index2).toInt();
      mag_thresh[CameraUnderWell][1] = mv.substring(index2 + 1, index - 1).toInt();
      mv = mv.substring(index + 1); // remove magic number

      index = mv.indexOf('#');
      index2 = mv.indexOf(',', index);
      post_thresh[CameraUnderWell][0] = mv.substring(1, index2).toInt();
      post_thresh[CameraUnderWell][1] = mv.substring(index2 + 1, index - 1).toInt();
      mv = mv.substring(index + 1); // remove magic number

      index = mv.indexOf('#');
      index2 = mv.indexOf(',', index);
      post_centroid[CameraUnderWell][0] = mv.substring(1, index2).toInt();
      post_centroid[CameraUnderWell][1] = mv.substring(index2 + 1, index - 1).toInt();
      mv = mv.substring(index + 1); // remove magic number
    }
    if (magic == -41) { // Camera reset
      CameraInited[CameraUnderWell] = LOW;
      waitForCameraInit = LOW;
    }
    if (magic == -51) { // Motor Initialization Handshake
      int index = mv.indexOf('#');
      int CameraUnderWell = mv.substring(0, index).toInt();
      if (MotorInitWell != CameraUnderWell) {
        motorInitState[MotorInitWell] = -1; // set motor init state to -1 (not initialized, do nothing)
      }
      else {
        beginMotorHomingFlag = HIGH;
        recievedMotorInitHandshake = HIGH;
        enableState[MotorInitWell] = LOW;
        // CameraInited[MotorInitWell] = HIGH;
      }
    }
    if (magic == -52) { // Motor Initialization Handshake (not initialized)
      int index = mv.indexOf("#");
      int CameraUnderWell = mv.substring(0, index).toInt();
      if (MotorInitWell != CameraUnderWell) {
        motorInitState[MotorInitWell] = -1;
      }
      else {
        OMV.print(CameraUnderWell);
        OMV.println("&INIT#");
        recievedMotorInitHandshake = HIGH;
        beginMotorHomingFlag = HIGH;
        enableState[MotorInitWell] = LOW;
        // CameraInited[MotorInitWell] = LOW;
      }

    }
  }

  comm_time = timer_comm;
  
  // MOTOR UPDATE
  for (int st = 0; st < 4; st++) {
    // enableState[st] = LOW;
    digitalWrite(EN[st], enableState[st]);

    if (timers[st] > period[st])  { // RESET TIMER IF PAST PERIOD LENGTH
      timers[st] = 0;   
    }
    if (firstCycle[st])           { // RESET TIMER IF FIRST CYCLE, SWITCH TO SECOND CYCLE
      timers[st] = 0;  
      firstCycle[st] = LOW;
    }
    if (enableState[st] == HIGH)  { // HOLD TIMER AT BEGINNING OF CYCLE IF LOW
      timers[st] = 0;  
    }


    
    if (enableState[st] == LOW) { // MOTOR IS ENABLED
      /*
      Order by priority: 
        1. Emergency retract
        2. Motor initialization (re-homing)
        3. Reset Motor position back to 0
        4. Manual override of position
        5. Normal cycling
      */
      // MotorInited[st] = HIGH;
      if (motorRetractFlag[st] == HIGH) { // EMERGENCY RETRACT
        stArray[st]->moveTo(-2000); // MOVE ARBITRARILY BACK
        stArray[st]->setMaxSpeed(5000); // SET HIGH SPEED
        stArray[st]->setAcceleration(10000); // SET HIGH ACCELERATION
        stArray[st]->run();
        if (stArray[st]->distanceToGo() == 0) { 
          motorRetractFlag[st] = LOW; 
          MotorResetPosition(st);
        }
      }
      
      else if (motorInitState[st] > 0) { // Do something if it's not 0 or -1
        switch (motorInitState[st]) {
          case 0: // Somehow got through if-else condition
            break;
          case 1: // Bring up 500 steps, before initialization
            if (CameraInited[st] == HIGH) {
              OMV.print(st);
              OMV.println("&MOTORINIT&HOME#");
              motorInitState[st] = 2;
            }
            else {
              motorInitState[st] = 0; // exit out of motor initialization immediately
            }
            break;
          case 2:
            stArray[st]->setMaxSpeed(3000);
            stArray[st]->setAcceleration(2000);
            stArray[st]->moveTo(-5000);
            stArray[st]->run();
            
            
            if (stArray[st]->distanceToGo() == 0) { 
              motorInitState[st] = 3; // Continue onto next phase
              stArray[st]->setCurrentPosition(0);
              // CameraInited[st] = LOW; // Force low
              OMV.print(st);
              OMV.println("&INIT#"); // RE-INITIALIZE CAMERA
              // OMV.print(st);
              // OMV.println("&MOTORINIT&HOME#");
              }
            break;
          case 3: // Bring down 3000 steps, fast
            stArray[st]->moveTo(3000);
            stArray[st]->setMaxSpeed(1000); // Fast speed
            stArray[st]->setAcceleration(3000); // Fast acceleration
            stArray[st]->run();
            if (stretchValue > 2 && gotStretchFlag==HIGH) { // as bring down, check for stretch value return
              stArray[st]->setCurrentPosition(0); // set current position to 0 (for relative position)
              motorInitState[st] = 5; // Continue onto next phase
              OMV.print(st);
              OMV.println("&MOTORINIT&HOME#");
            }
            if (stArray[st]->distanceToGo() == 0) { // if reached 3000 steps, still continue down, but slower
              motorInitState[st] = 4;
              stArray[st]->setCurrentPosition(0);

            }
            break;
          case 4: // Bring down arbitrarily, monitor for stretch
            stArray[st]->moveTo(750); // Arbitrarily down
            stArray[st]->setMaxSpeed(50); // Slow speed
            stArray[st]->setAcceleration(50); // Matching acceleration
            stArray[st]->run();
            if (stretchValue > 2 && gotStretchFlag==HIGH) { // as bring down, check for stretch value return
              stArray[st]->setCurrentPosition(0); // set current position to 0 (for relative position)
              motorInitState[st] = 5; // Continue onto next phase
              OMV.print(st);
              OMV.println("&MOTORINIT&HOME#");
            }
            else if (stArray[st]->distanceToGo() == 0) { // If reached arbitrarily low destination (too far)
              MotorRetract(st); // Retract Motor
              motorInitState[st] = 0; // Stop Motor Initialization
              OMV.print(st);
              OMV.println("&MOTORINIT&FAIL#");
            }
            
            break;
          case 5: // Bring up 200 steps (somewhat arbitrary), monitor for lack of stretch
            stArray[st]->setMaxSpeed(25); // slow
            stArray[st]->setAcceleration(25); // matching acceleration
            stArray[st]->moveTo(-200); // stop at -200 (in case of error)
            stArray[st]->run();
            
            if (gotStretchFlag == HIGH && stretchValue < 0.5) {
              enableState[st] = HIGH; // DISABLE MOTOR
              MotorInited[st] = HIGH; // MOTOR INITIALIZED
              MotorResetPosition(st);
              stArray[st]->setAcceleration(100000);
              motorInitState[st] = -1;
              OMV.print(st);
              OMV.println("&MOTORINIT&PASS#");
            }
            else if (stArray[st]->distanceToGo() == 0) { // FAILED MOTOR INIT
              MotorInited[st] = LOW;
              enableState[st] = HIGH;
              MotorResetPosition(st);
              stArray[st]->setAcceleration(100000);
              motorInitState[st] = 0;
              OMV.print(st);
              OMV.println("&MOTORINIT&FAIL#");
            }
            break;
          default:
            break;
        }
        gotStretchFlag = LOW;
      }

      else if (resetFlag[st] == HIGH) { // MOTOR RESET
        stArray[st]->setMaxSpeed(1000); // quick
        stArray[st]->moveTo(0); // move back to 0 position
        stArray[st]->run();
        if (stArray[st]->distanceToGo() == 0) { // when reached final destination
          enableState[st] = HIGH; // disable motor
          resetFlag[st] = LOW; // turn off resetting
          timers[st] = 0; // reset timer
        }
      }
      else if (manualOverride[st] == HIGH) { // MANUAL OVERRIDE OF POSITION (RUN AS FAST TO DESIGNATED POSITION)
        stArray[st] -> run(); // ASSUMED TO BE MANUAL MOTOR OVERRIDE
        if (stArray[st]->distanceToGo() == 0) { // WHEN REACHED FINAL DESTINATION
          manualOverride[st] = LOW; // turn off manual override
          enableState[st] = HIGH; // disable motor
        }
      }
      else { //if (MotorInited[st] == HIGH) {
        int inPhase = 4;
        inPhase = checkPhase(timers[st], ts[st]);
        stArray[st]->setMaxSpeed(abs(speedPhase[st][inPhase - 1])); // set speed to phase speed
        stArray[st]->moveTo(locPhase[st][inPhase]); // set position to phase destination
        stArray[st]->run();
      }
    }

  }

  // CAMERA UPDATE
  if (stCamera.distanceToGo() != 0) {
    enableStateCamera = HIGH; digitalWrite(ENCAMERA, !enableStateCamera); digitalWrite(SLPCAMERA, enableStateCamera);
    MovingCamera = HIGH;
    stCamera.run();
  }
  else {
    MovingCamera = LOW;
    if (ResetCameraFlag == HIGH) {
      ResetCameraFlag = LOW;
      stCamera.setAcceleration(CameraAcceleration);
      stCamera.setMaxSpeed(CameraSpeed);
      stCamera.setCurrentPosition(-1*CameraStartingPosition);
      CameraMove(0);
      CameraUnderWell = 0;
      OMV.print(CameraUnderWell);
      OMV.println("&CHANGE#");
    }
    else if (ResetCameraFlag == LOW) {
      stCamera.setAcceleration(CameraAcceleration);
      stCamera.setMaxSpeed(CameraSpeed);
    }
    enableStateCamera = LOW; digitalWrite(ENCAMERA, !enableStateCamera); digitalWrite(SLPCAMERA, enableStateCamera);
  }

  gotStretchFlag = LOW;
  // TIMER UPDATE
  if (timerSerial > 25 * 1000) {
    if (HELPERFLAG == HIGH) {
      Serial.flush();
      Serial.print("HELPER");
      for (int idx = 0; idx < k; idx++) {

        Serial.print('&');
        Serial.print(valueArray[idx]);

      }
      Serial.println('#');

    }
    else {
      Serial.flush();
      Serial.print("-33;");
      Serial.print(millis()); // Arduino Time
      Serial.print(';');
      Serial.print(t_camera);
      Serial.print('&');
      Serial.print(CameraUnderWell);
      Serial.print('&');
      Serial.print(MotorInitWell);
      Serial.print('&');
      Serial.print(recievedMotorInitHandshake);
      Serial.print('&');
      Serial.print(stretchValue);
      Serial.print('&');
      Serial.print(foundMax);
      Serial.print('&');
      Serial.print(maxs);
      Serial.print(';');
      

      for (int st = 0; st < 4; st++) {
        //      Serial.print(timers[st]*100/period[st]);
        //      Serial.print(',');
        Serial.print(st);
        Serial.print('&');
        Serial.print(timers[st] / 1000);
        Serial.print('&');
        Serial.print(stArray[st]->currentPosition());
        Serial.print('&');
        Serial.print(dists[st]);
        Serial.print('&');
        Serial.print(freqs[st]);
        Serial.print('&');
        Serial.print(passive_len[st]);
        Serial.print('&');
        Serial.print(MotorInited[st]);
        Serial.print('&');
        Serial.print(CameraInited[st]);
        Serial.print('&');
        Serial.print(enableState[st]);
        Serial.print('&');
        Serial.print(motorInitState[st]);
        Serial.print(';');
        //            Serial.print(enableState[st]);
      }
      if   (MovingCamera == HIGH) {
        Serial.print(-1);
        digitalWrite(10, HIGH);
      }
      else                        {
        Serial.print(rowLabel[CameraUnderWell]);
        digitalWrite(10, LOW);
      }
      Serial.print('&');
      Serial.print(digitalRead(10));
      Serial.print('&');
      Serial.print(stCamera.currentPosition());
      Serial.print('&');
      Serial.print(CameraUnderWell);
      Serial.print('&');
      Serial.print(gotStretchFlag);
      Serial.print('&');
      Serial.print(algorithm_magnet_flag);
      Serial.print('&');
      Serial.print(timer_comm);
      Serial.println(' ');
      timerSerial = 0;
    }
  }
}