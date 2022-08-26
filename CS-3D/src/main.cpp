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
const int STEP[4] = {23, 22, 21, 20};
const int DIR[4]  = {18, 17, 16, 15};
const int EN[4]   = { 6,  5,  4,  3};

// MOTOR VARIABLES
float                freqs[4] = { 1,  1,  1,  1};
uint32_t            period[4] = {   0 ,    0 ,    0 ,     0};
long                 dists[4] = {  25,   25,   25,   25};
int MotorStartingPositions[4] = { 500,  500,  500,  500};

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
int     CameraUnderWell         = 1;
boolean MovingCamera            = LOW;
int 	  CameraSpeed 		        = 2500;
int     CameraPosition          = 0;
boolean enableStateCamera       = LOW; // LOW to disable
boolean ResetCameraFlag         = HIGH;
char    rowLabel[4]             = {'A', 'B', 'C', 'D'};
boolean POSTCENTROIDMANUALFLAG  = LOW;


// TIMING VARIABLES
elapsedMicros timers[4];
elapsedMicros timerSerial;

// COMMUNICATION VARIABLES
String command;
String printstring;
String mv;
boolean recievedHandshake = LOW;
char serial1buffer[2000];
int sizeOfSerialBuffer;

// OPENMV VARIABLES
float stretchValue = 0.;
float frequencyValue = 0.;
float t_camera = 0.;
float last_max_stretch = 0.;
int passive_len[4] = {100,100,100,100};
int mag_thresh[4][2] = {{20,40}, {20,40}, {20,40}, {20,40}};
int post_thresh[4][2] = { {0, 15}, {0, 15}, {0, 15}, {0, 15}};
int post_centroid[4][2] = { {0,0}, {0,0}, {0,0}, {0,0}};
boolean HELPERFLAG = LOW;
String HELPERMASK = "None";
int32_t valueArray[50];
int k=0;
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
void MotorEnable(int MOTOR) { // ENABLE MOTOR
  enableState[MOTOR] = !enableState[MOTOR];
  if      (enableState[MOTOR] == HIGH) { resetFlag[MOTOR] = HIGH; enableState[MOTOR] = LOW;}
  else if (enableState[MOTOR] == LOW)  { manualOverride[MOTOR] = LOW; timers[MOTOR] = 0;}
}
void MotorDistance(int MOTOR, float d) { // CHANGE DISTANCE OF CYCLING
  dists[MOTOR] = d;
  for (int ph = 1; ph < 3; ph++) { locPhase[MOTOR][ph] = dists[MOTOR]; }
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
void MotorManual(int MOTOR, int m) { // MANUAL OVERRIDE OF MOTOR POSITION
  manualOverride[MOTOR] = HIGH;
  enableState[MOTOR] = LOW;
  stArray[MOTOR]->moveTo(m);
  stArray[MOTOR]->setMaxSpeed(1000);
  if (stArray[MOTOR]->distanceToGo() == 0) {
    enableState[MOTOR] = HIGH;
  }
  
}
void MotorResetPosition(int MOTOR) { // RESET MOTOR POSITION
  stArray[MOTOR]->setCurrentPosition(0);
  enableState[MOTOR] = HIGH;
}
void WaveformUpdate(uint32_t RIS, uint32_t HOL, uint32_t FAL) { // CHANGE WAVEFORM SHAPE
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
  stArray[MOTOR]->setMaxSpeed(1000);
  stArray[MOTOR]->moveTo(-7000);
}
void CameraMove(int CameraPosition) {
//  enableStateCamera = HIGH;
//  ResetCameraFlag = LOW;
//  digitalWrite(SLPCAMERA, enableStateCamera);
  stCamera.moveTo(CameraPosition); 
}
void CameraReset() {
  Serial.print("RESET IN");
  ResetCameraFlag = HIGH;
//  enableStateCamera = HIGH;
//  digitalWrite(SLPCAMERA, enableStateCamera);
  CameraMove(-23350);
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
  // MOTOR VARIABLE SET-UP
  for (int st = 0; st < 4; st++) {
    pinMode(EN[st], OUTPUT); digitalWrite(EN[st], enableState[st]);


    stArray[st] = new AccelStepper(1, STEP[st], DIR[st]);
    stArray[st]->setMaxSpeed(10000);
    stArray[st]->setAcceleration(100000);
    stArray[st]->setPinsInverted(HIGH);

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
  stCamera.setMaxSpeed(CameraSpeed);
  stCamera.setAcceleration(CameraSpeed);
  stCamera.setCurrentPosition(0);
  ResetCameraFlag = LOW;
  pinMode(SLPCAMERA, OUTPUT); digitalWrite(SLPCAMERA, enableStateCamera);
  pinMode (ENCAMERA, OUTPUT); digitalWrite(ENCAMERA, !enableStateCamera);
  // COMMUNICATION SET-UP
  Serial.begin(9600);
  Serial.setTimeout(0);
  
  OMV.begin(9600); // RX: 0, TX: 1
  // OMV.setTimeout(0);
  sizeOfSerialBuffer = sizeof(serial1buffer);
  OMV.addMemoryForRead(serial1buffer, sizeOfSerialBuffer);

  establishConnection();

  // MOTOR INITIALIZATION
  for (int st = 0; st < 4; st++) {
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
    MotorManual(st, MotorStartingPositions[st]);
  }

  allDistanceToGo = 0;
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
	enableState[st] = HIGH;
	manualOverride[st] = LOW;
	resetFlag[st] = HIGH;
  }
}
String partialReturn(char del) {
  int index = mv.indexOf('&');
  String val = mv.substring(0, index);
  mv = mv.substring(index+1);
  return val;
}

void loop() {
  // COMMUNICATION UPDATE
  if (Serial.available() > 0) { // FROM COMPUTER
    command = Serial.readString();
    // Serial.println(command.substring(0, command.length() - 1));
    Serial.flush();

    if (command.substring(0, 16) == "POSTMANUALTOGGLE") {
      POSTCENTROIDMANUALFLAG = !POSTCENTROIDMANUALFLAG;
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("POSTMANUALTOGGLE");
      OMV.print('&');
      OMV.print(POSTCENTROIDMANUALFLAG);
      OMV.print('&');
      OMV.println(' ');
      Serial.print('#');


    }
    else if (command.substring(0, 10) == "POSTMANUAL") {
      int del = command.indexOf(',');
      int PostCentroidx = command.substring(10, del).toInt();
      int PostCentroidy = command.substring(del+1, command.length()-1).toInt();
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("POSTMANUAL");
      OMV.print('&');
      OMV.print(PostCentroidx);
      OMV.print('&');
      OMV.print(PostCentroidy);
      OMV.print('&');
      OMV.println(' ');
      Serial.print('#');

      
    }
    // CAMERA COMMANDS PASS THROUGH TO OPENMV 
    else if (command.substring(0, 4) == "INIT") {
      // camera initialization routine
      //CameraUnderWell = command.substring(5,6).toInt();
      if (enableState[CameraUnderWell-1] == LOW) {
        MotorEnable(CameraUnderWell-1);
        // MAYBE: wait for motor to fully retract, without pausing other motors
      }
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("INIT");
      Serial.println('#');
    }
    else if (command.substring(0, 4) == "POST") {
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("POST");
      Serial.println('#');
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
      int upper = command.substring(index+1, command.length()-1).toInt();
      // String printstring = CameraUnderWell + "&THRESH&" + lower + '&' + upper;
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("THRESH");
      OMV.print('&');
      OMV.print(lower);
      OMV.print('&');
      OMV.print(upper);
      Serial.println('#');
      // OMV.println(printstring);
      // Serial.println(input1+printstring);
    }
    else if (command.substring(0, 12) == "HELPERTOGGLE") {
      HELPERFLAG = !HELPERFLAG;
      OMV.print(CameraUnderWell);
      OMV.print("&HELPERTOGGLE&");
      OMV.print(HELPERFLAG);
      OMV.println('#');

      Serial.println("Toggled Helper.");

    }
    else if (command.substring(0, 10) == "HELPERMASK") {
      String mask = command.substring(11, command.length()-1);
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
    else if (command.substring(0, 6) == "HELPER") {
      command = command.substring(6);
      Serial.println(command);
      
      int i=0, del=0;

      for (int idx = 0; idx < 50; idx++){
          valueArray[idx] = -55; // reset default to -55
      }
      k = 0;
      while (del != -1) {
        // Serial.println("in parsing loop");
        del = command.indexOf(',');
        valueArray[i] = command.substring(0,del).toInt();
        command = command.substring(del+1);
        k=k+1;
        i = i + 1;
      }
      // for( int idx = 0; idx < 50; idx++) {
      //   Serial.print(valueArray[idx]);
      //   Serial.print(',');
      // }
      // Serial.println(' ');
      OMV.print(CameraUnderWell);
      OMV.print("&HELPER");

      Serial.print(CameraUnderWell);
      Serial.print("&HELPER");
      for (int idx = 0; idx < k; idx++) {
        OMV.print('&');
        OMV.print(valueArray[idx]);
        Serial.print('&');
        Serial.print(valueArray[idx]);
        
      }
      OMV.println('#');
      Serial.println('#');

    }
    // MOTOR COMMANDS
    else if (command.substring(0, 1) == "M") { // enable/disable Motor
      int st = command.substring(1, 2).toInt() - 1;
      MotorEnable(st);
    }
    else if (command.substring(0, 1) == "D") { // change motor Distance
      int st = command.substring(1, 2).toInt() - 1;
      float d = command.substring(3, command.length() - 1).toFloat();
      MotorDistance(st, d);
    }
    else if (command.substring(0, 1) == "F") { // change motor Frequency
      int st = command.substring(1, 2).toInt() - 1;
      float f = command.substring(3, command.length() - 1).toFloat();
      MotorFrequency(st, f);
    }
    else if (command.substring(0, 1) == "O") { // position Override
      int st = command.substring(1, 2).toInt() - 1;
      int m = command.substring(3, command.length() - 1).toInt();
      MotorManual(st, m);
    }
    else if (command.substring(0, 1) == "R") { // Reset
      int st = command.substring(1,2).toInt() - 1;
      MotorResetPosition(st);
    }
    else if (command.substring(0, 1) == "S") { // waveform Shape
      uint32_t RIS = command.substring(1,3).toInt();
      uint32_t HOL = command.substring(4,6).toInt();
      uint32_t FAL = command.substring(7,command.length() - 1).toInt();
      WaveformUpdate(RIS, HOL, FAL);
    }
    else if (command.substring(0, 1) == "A") { // motor Adjustment
      int st = command.substring(1, 2).toInt() - 1;
      int ADJ = command.substring(3, command.length() - 1).toInt();
      MotorAdjust(st, ADJ);
    }

    // CAMERA STAGE COMMANDS
    else if (command.substring(0, 1) == "C") { // Camera move
      CameraUnderWell = command.substring(1,2).toInt();
      CameraPosition = command.substring(3,command.length()-1).toInt();
      OMV.print(CameraUnderWell);
      OMV.print('&');
      OMV.print("CHANGE");
      
      Serial.println('#');
      
      CameraMove(CameraPosition);
      CameraMove(CameraPosition);
    }
    else if (command.substring(0, 1) == "B") {
      CameraSpeed = command.substring(1,command.length()-1).toInt();
      stCamera.setMaxSpeed(CameraSpeed);
      stCamera.setAcceleration(CameraSpeed);
  
    }
    else if (command.substring(0, 1) == "V") { // camera reset
      CameraReset();
//      Serial.println("RESET");
    }
    else if (command.substring(0, 1) == "X") { // emergency retract
      int st = command.substring(1, command.length() - 1).toInt() - 1;
      MotorRetract(st);
    }
    else if (command.substring(0, 1) == "P") { // starting Positioning
      int st = command.substring(1, 3).toInt() - 1;
      int action = command.substring(4, command.length() - 1).toInt();
      StartingPositions(st, action);
    }
  }

if (OMV.available() > 0) { // FROM OPENMV
    //    val = OMV.read
    mv = OMV.readStringUntil('\n');
    OMV.flush();
    // Serial.println(mv);
    
    int index = mv.indexOf('&'); // find first &
    int magic = mv.substring(0, index).toInt(); // store first substring as magic number
    mv = mv.substring(index+1); // remove magic number
    // Serial.print(magic);
    if (magic == -35){ // <t>&<stretch>&<max_stretch>&<current_well>\n
      // Serial.println(mv);

      index = mv.indexOf('&');
      t_camera = mv.substring(0, index).toFloat();
      mv = mv.substring(index+1);

      index = mv.indexOf('&');
      stretchValue = mv.substring(0, index).toFloat();
      mv = mv.substring(index+1); // remove magic number

      index = mv.indexOf('&');
      last_max_stretch = mv.substring(0, index).toFloat();
      mv = mv.substring(index+1); // remove magic number
      
      // index = mv.indexOf('&');
      // CameraUnderWell = mv.substring(0, index).toInt();
      // mv = mv.substring(index+1); // remove magic number

      // Serial.println(stretchValue);
    }

    if (magic == -43) { // <t>&<current_well>&<passive_length>&<magnet_thresh>&<post_threshold>&<centroid_post>\n
      index = mv.indexOf('&');
      t_camera = mv.substring(0, index).toFloat();
      mv = mv.substring(index+1);

      index = mv.indexOf('&');
      CameraUnderWell = mv.substring(0, index).toInt();
      mv = mv.substring(index+1);

      
      index = mv.indexOf('&');
      passive_len[CameraUnderWell-1] = mv.substring(0, index).toInt();
      mv = mv.substring(index+1); // remove magic number

      index = mv.indexOf('&');
      int index2 = mv.indexOf(',', index);
      mag_thresh[CameraUnderWell-1][0] = mv.substring(1, index2).toInt();
      mag_thresh[CameraUnderWell-1][1] = mv.substring(index2+1, index-1).toInt();
      mv = mv.substring(index+1); // remove magic number

      index = mv.indexOf('&');
      index2 = mv.indexOf(',', index);
      post_thresh[CameraUnderWell-1][0] = mv.substring(1, index2).toInt();
      post_thresh[CameraUnderWell-1][1] = mv.substring(index2+1, index-1).toInt();
      mv = mv.substring(index+1); // remove magic number

      index = mv.indexOf('&');
      index2 = mv.indexOf(',', index);
      post_centroid[CameraUnderWell-1][0] = mv.substring(1, index2).toInt();
      post_centroid[CameraUnderWell-1][1] = mv.substring(index2+1, index-1).toInt();
      mv = mv.substring(index+1); // remove magic number
    }
    // Serial.println(mv);
    
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
        stArray[st]->setMaxSpeed(1000);
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
        if (stArray[st]->currentPosition() == -7000) {
          MotorResetPosition(st);
          MotorManual(st, 500);
        }
		if (stArray[st]->distanceToGo() == 0) {
			manualOverride[st] = LOW;
			enableState[st] = HIGH;
		}
        
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
      stCamera.setCurrentPosition(-3200);
      CameraMove(0);
      // CameraUnderWell = 1;
    }
    enableStateCamera = LOW; digitalWrite(ENCAMERA, !enableStateCamera); digitalWrite(SLPCAMERA, enableStateCamera);
  }
  



  
  // TIMER UPDATE
  if (timerSerial > 10 * 1000) {

    if (HELPERFLAG==HIGH) {
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
      Serial.print("-33");
      Serial.print(',');
      Serial.print(millis()); // Arduino Time
      Serial.print(',');
      Serial.print(t_camera);
      Serial.print('&');
      Serial.print(CameraUnderWell);
      Serial.print('&');
      Serial.print(stretchValue);
      Serial.print(',');
      
      for (int st = 0; st < 4; st++) {
        //      Serial.print(timers[st]*100/period[st]);
        //      Serial.print(',');
        Serial.print(st+1);
        Serial.print('&');
        Serial.print(timers[st]/1000);
        Serial.print('&');
        Serial.print(stArray[st]->currentPosition());
        Serial.print('&');
        Serial.print(dists[st]);
        Serial.print('&');
        Serial.print(freqs[st]);
        Serial.print('&');
        Serial.print(enableState[st]);
        Serial.print('&');
        Serial.print(manualOverride[st]);
        Serial.print(',');
  //            Serial.print(enableState[st]);
      }
      if   (MovingCamera == HIGH) { Serial.print(-1); digitalWrite(10, HIGH);}
      else                        { Serial.print(rowLabel[CameraUnderWell-1]);  digitalWrite(10, LOW);}
      Serial.print('&');
      Serial.print(digitalRead(10));
      Serial.print('&');
      Serial.print(stCamera.currentPosition());
      Serial.print('&'+printstring);
      
      Serial.println(' ');
      timerSerial = 0;
    }
  }
}