/*******************************************************************************
* Copyright 2023 Patrick M. Pilarski (pilarski@ualberta.ca)
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

// Implements on Wrystlebot: Continual learning via TD-Learning, General Value Functions, Pavlovian signalling and control
// Hardware: Arduino MKR WiFi 1010, IMU shield, RGB shield, Dynamixel shield, prototyping shield x 1
// Other sensors and actuators connected: Dynamixel XL-330-M288-T (to Dynamixel shield), 2-button interface (to input pins D7&D8) 

#include <ArduinoGraphics.h>
#include <Arduino_MKRRGB.h>

#include <DynamixelShield.h>
#include <MKRIMU.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

float servoPos;
float servoVel;
float servoCurr;

const int idxROLL = 0;
const int idxPTICH = 1;
const int idxYAW = 2;
const int idxSPOS = 3; 
const int idxSLOAD = 5;
const int idxSVEL = 4;
const int idxC = 6;
const int idxTCT = 7;
const int idxPRED1 = 8;
const int idxSURPRISE1 = 9;
const int idxPRED2 = 10;
const int idxSURPRISE2 = 11;

float EMG = 0;

float c = 0;
float tct = 0;
float tctDecay = 0.95;
float pred1 = 0;
float surprise1 = 0;
float pred2 = 0;
float surprise2 = 0;

float target = 0;
float targetDelta = 0;
int colourRedBoost = 0;
int colourGreenBoost = 0;
int colourBlueBoost = 0;
const int stateRows = 7;
const int stateCols = 12;
int state[stateRows][stateCols];

const int numFeatures = 20;
float w[numFeatures];
float xtp1[numFeatures];
float xt[numFeatures];
float e[numFeatures];

float w2[numFeatures];
float xtp12[numFeatures];
float xt2[numFeatures];
float e2[numFeatures];

float delta = 0;
float lambda = 0.3;
float alpha = 0.1;
float discount = 0.9;

int counter = 0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  // initialize the display
  MATRIX.begin();

  // set the brightness, supported values are 0 - 255
  MATRIX.brightness(25);

  if (!IMU.begin()) {
    MATRIX.text("!IMU", 0, 1);

    while (1);
  }

  // For Buttons
  pinMode(7, INPUT_PULLUP);  // Pin 7 reads 1 if not connected to GND
  pinMode(8, INPUT_PULLUP);  // Pin 7 reads 1 if not connected to GND 

  // For Feedback Circuit
  pinMode(4, OUTPUT);  //  Pin 4 Vibration Motor

  // For EMG
  pinMode(A5, INPUT_PULLUP);
  //pinMode(A5, INPUT);

  // For LED
  pinMode(LED_BUILTIN, OUTPUT);

  for(int i = 0; i < numFeatures; i++)
  {
    xt[i] = 0;
    xtp1[i] = 0;  
    xt2[i] = 0;
    xtp12[i] = 0;
    w[i] = 0;
    w2[i] = 0; 
    e[i] = 0;
    e2[i] = 0;         
  }

}

void loop() {

  float heading, roll, pitch;
  IMU.readEulerAngles(heading, roll, pitch);
  EMG = analogRead(A5);

  counter += 1;

  colourRedBoost = 0;
  colourGreenBoost = 0;
  colourBlueBoost = 0;

  // EMG Control Updates
  if (EMG > 100) {
    targetDelta = int(EMG / 4);
  }
  else {
    // "Default closed" operation
    targetDelta = -200;  
    // To align with target of load prediction,
    // Do not close to the point of high load
    if (target + targetDelta < -100) {
    targetDelta = 0;
    }
  }

  // Read button contorls pins & move servo
  // This overrides the EMG as needed
  if (!digitalRead(7)) {
    targetDelta = -300;
  }
  if (!digitalRead(8) ) {
    targetDelta = 300;
  }

  // Update target position of gripper
  target += targetDelta;

  // Boundary check
  if (target > 1000) {
    target = 1000;
  }
  if (target < -300) {
    target = -300;
  }
  dxl.setGoalPosition(DXL_ID, 1000+target);

  digitalWrite(LED_BUILTIN, !digitalRead(7) && !digitalRead(8));

  // Set cumulant to double button press
  c = !digitalRead(7) && !digitalRead(8);

  // --- State Update Function ---
  if (counter > 5) {
    counter = 0;
    for (int j=0; j < stateCols; j++) {
      for (int i = stateRows-1; i > 0; i--) {
        state[i][j] = state[i-1][j]; 
      }
    }
  }

  // Update and decay tile-coded trace (c.f., Pilarski et al 2022 / Butcher et al. 2022 / Brenneis et al. 2022, Frost Hollow Experiments)
  tct=tct*tctDecay;

  if (c > 0) {
    tct = c;
    colourRedBoost=75;
    colourGreenBoost=75;  
    colourBlueBoost=0;
  }

  // Sample motor vaules
  servoPos = max(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)-90,0);
  servoCurr = dxl.getPresentCurrent(DXL_ID, UNIT_PERCENT);
  servoVel = dxl.getPresentVelocity(DXL_ID, UNIT_PERCENT);

  // Scale and update state for display on RGB shield
  state[0][idxROLL] = int(abs(3*roll));
  state[0][idxPTICH] = int(abs(3*pitch));
  state[0][idxYAW] = int(abs(heading));
  state[0][idxSPOS] = int(servoPos*3);
  state[0][idxSLOAD] = int(abs(servoCurr)*25);
  state[0][idxSVEL] = int(servoVel*20+100);
  state[0][idxTCT] = int(tct * 255);
  state[0][idxC] = int(c *255);
  state[0][idxPRED1] = 0;
  state[0][idxSURPRISE1] = 0;
  state[0][idxPRED2] = 0;
  state[0][idxSURPRISE2] = 0;

  // --- TD-Lambda learning code for GVF1 (user-delivered time-based cue prediction) ---
  float wTxt = 0;
  float wTxtp1 = 0; 
  int idx = 0;
  int idxViz = 0;
  for(int i = 0; i < numFeatures; i++)
  {
    xt[i] = xtp1[i];
    xtp1[i] = 0;  
  }

  idx = int(tct*20); 
  idxViz = idx;
  xtp1[idx] = 1;

  for(int i = 0; i < numFeatures; i++)
  {
    wTxt += xt[i] * w[i]; 
    wTxtp1 += xtp1[i] * w[i];   
  }

  delta = c*10 + discount*wTxtp1 - wTxt;
  for(int i = 0; i < numFeatures; i++)
  {
    e[i] = min(lambda*discount*e[i] + xt[i],1);
    w[i] += delta*alpha*xt[i];
  }
  
  pred1 = wTxtp1;
  surprise1 = delta;
  state[0][idxPRED1] = pred1 * 20;
  state[0][idxSURPRISE1] = surprise1 * 20;

  // --- TD-Lambda learning code for GVF2 (servo load prediction) ---
  wTxt = 0;
  wTxtp1 = 0; 

  for(int i = 0; i < numFeatures; i++)
  {
    xt2[i] = xtp12[i];
    xtp12[i] = 0;
  }

  idx = int(servoPos/5); 
  xtp12[idx] = 1;

  for(int i = 0; i < numFeatures; i++)
  {
    wTxt += xt2[i] * w2[i]; 
    wTxtp1 += xtp12[i] * w2[i];   
  }

  float c2 = 0;
  if (abs(servoCurr) > 10) {
    c2 = 1;
  }
  delta = c2 + discount*wTxtp1 - wTxt;
  for(int i = 0; i < numFeatures; i++)
  {
    e2[i] = min(lambda*discount*e2[i] + xt2[i],1);
    w2[i] += delta*alpha*e2[i];
  }

  pred2 = wTxtp1;
  surprise2 = delta;
  state[0][idxPRED2] = pred2 * 20;
  state[0][idxSURPRISE2] = surprise2 * 20;

  // --- Draw values on screen ---

  if (state[0][idxPRED1] > 0) {
    colourGreenBoost += pred1 * 20;   
    if (pred1 * 20 > 50) {
      analogWrite(4, int(pred1 * 50));     
    }
    else {
      analogWrite(4, 0); 
    } 
  }

  if (state[0][idxPRED2] > 0) {
    colourBlueBoost += pred2 * 50;  
  }

  // TODO: when UDE is implemented
  // if (state[0][idxSURPRISE2] > 0) {
  //   colourRedBoost += 10 * state[0][idxSURPRISE2] ;
  //   colourBlueBoost += 20 * state[0][idxSURPRISE2] ;
  // }

  // FOR EMG... borrowing the SURPRISE2 Measure
  // TODO: refactor to show EMG properly in variable names
  state[0][idxSURPRISE2] = int(EMG / 4.0); 

  MATRIX.beginDraw();
  MATRIX.clear();
  
  // >>> MAIN display <<<
  // Display a memory buffer of main signals sampled by the board
  // and also alter display in response to precitions and cumulant
  for (int i=0; i < stateRows; i++) {
    for (int j=0; j < stateCols; j++) {
      int cbR = 0;
      int cbG = 0;
      int cbB = 0;
      if (j < 3) {cbR = colourRedBoost; cbG = colourGreenBoost + 100-i*20; cbB = colourBlueBoost + 100-i*20;}
      if (j > 2) {cbR = colourRedBoost; cbG = colourGreenBoost; cbB = colourBlueBoost + 200-i*40;}
      if (j > 5) {cbR = colourRedBoost + 100 - i*20; cbG = colourGreenBoost + 100-i*20; cbB = colourBlueBoost;}
      if (j > 7) {cbR = colourRedBoost + 100 - i*20; cbG = colourGreenBoost; cbB = colourBlueBoost + 100-i*20;}        
      if (j > 9) {cbR = colourRedBoost + 70 - i*10; cbG = colourGreenBoost; cbB = colourBlueBoost + 200-i*40;}
      int cR = min(255,max(state[i][j] + cbR, 0)); 
      int cG = min(255,max(state[i][j] + cbG, 0)); 
      int cB = min(255,max(state[i][j] + cbB, 0));             
      MATRIX.set(j, i, cR, cG, cB); // X, Y, and R, G, B
    }
  }

  // >>> STAE AND WEIGHT alternate display <<<
  // // Debugging: show feature and weight vetors if small enough 
  // for (int j=0; j < numFeatures; j++) {          
  //   MATRIX.set(j, 0, xt[j]*100, 0, 0); // X, Y, and R, G, B
  //   MATRIX.set(j, 1, xt[j]*100, 0, 0); // X, Y, and R, G, B
  //   MATRIX.set(j, 2, xt[j]*100, 0, 0); // X, Y, and R, G, B        
  // }
  // for (int j=0; j < numFeatures; j++) {       
  //   if (w[j] > 3) {
  //     MATRIX.set(j, 3, 0, 150, 0); // X, Y, and R, G, B
  //   }   
  //   MATRIX.set(j, 4, w[j]*50, w[j]*50, w[j]*50); // X, Y, and R, G, B
  //   MATRIX.set(j, 5, w[j]*50, w[j]*50, w[j]*50); // X, Y, and R, G, B
  //   MATRIX.set(j, 6, w[j]*50, w[j]*50, w[j]*50); // X, Y, and R, G, B        
  // }

  // >>> DUBUGGING data Point display <<<
  // MATRIX.stroke(255, 255, 255);
  // MATRIX.text(String(digitalRead(6)), 0, 1);
  // MATRIX.text(String(abs(servoCurr)), 0, 1);


  MATRIX.endDraw();

  // Loop time delay
  delay(100);
}

