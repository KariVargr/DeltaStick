#include <Adafruit_NeoPixel.h>
#include <HID-Project.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP23017.h>
#include <math.h>
#include "DeltaCode.h"

Adafruit_MCP23017 mcp;
DELTACAL deltaCal;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(0, 6, NEO_GRB + NEO_KHZ800);

float deadzone = 2.5;
float TZero = 97.00;
float Zshift = 121;

float xHome, yHome, zHome;
float xTheta, yTheta, zTheta;
float xValue, yValue, zValue;
float xT_HOME, yT_HOME, zT_HOME;

uint8_t xMaster, yMaster, zMaster;

float xMin, yMin, zMin, xMax, yMax, zMax;

int returnPower = 180;

//Setup Pins
//I2C in USE = 2, 3
//A2-A3 Free
int xTValue, yTValue;
float USBdeadzone = 10;
float xTCorrect = -10;
float yTCorrect = +42;

bool Debug = false;

int lastDEBUG = 0;
int lastUSB = 0;

int DEBUGDelay = 10;
int USBDelay = 10;

int deltaMode = 0;

int MOTORDIR[9] = {1,0, 14,15, 7,8 , 10,9,5};

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Gamepad.begin();
  mcp.begin();
  mcp.pinMode(0, INPUT);//Thumb
  mcp.pinMode(1, INPUT);//10
  mcp.pinMode(2, INPUT);//11
  mcp.pinMode(3, INPUT);//9
  mcp.pinMode(4, INPUT);//12
  mcp.pinMode(5, INPUT);//6
  mcp.pinMode(6, INPUT);//4
  mcp.pinMode(7, INPUT);//3
  mcp.pinMode(8, INPUT);//2
  mcp.pinMode(10, INPUT);//1
  mcp.pinMode(12, INPUT);//7
  mcp.pinMode(13, INPUT);//8
  mcp.pinMode(14, INPUT);//5
  mcp.pinMode(15, INPUT);//Trigger

  pinMode(A3, INPUT); //Thumb X
  pinMode(A6, INPUT); //Thumb Y
  
  pinMode(A0, INPUT); //DELTA X
  pinMode(A1, INPUT); //DELTA Y
  pinMode(A2, INPUT); //DELTA Z

  pinMode(6, INPUT); //DELTA Modes
  pinMode(16, INPUT); //DELTA Modes

  for(int i=0; i<9; i++){
    pinMode(MOTORDIR[i], OUTPUT);
    if(i <= 5){
      digitalWrite(MOTORDIR[i], LOW);
    }else{
      analogWrite(MOTORDIR[i], returnPower);
    }
  }
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  //DELTA MODES
    if(digitalRead(6) == 0 && digitalRead(16) == 1){
      deltaMode = 2;
    }else if(digitalRead(6) == 1 && digitalRead(16) == 0){
      deltaMode = 1;
    }else{
      deltaMode = 0;
    }
  //THUMB STICK
    xTValue = analogRead(A3)+xTCorrect;
    yTValue = analogRead(A6)+yTCorrect;
    
  //DELTA
    xTheta = round(analogRead(A0))+0;
    yTheta = round(analogRead(A1))+4;
    zTheta = round(analogRead(A2))+0;

    deltaCal.ForwardKinematic(xTheta-TZero, yTheta-TZero, zTheta-TZero, xValue, yValue, zValue);

    xMaster = map(constrain(round(xValue), -100, 100), -100, 100, 0, 255); // +- 100
    yMaster = map(constrain(round(yValue), -100, 100), -100, 100, 0, 255); // +- 100
    zMaster = map(constrain(round(zValue + Zshift), -45, 46), 46, -45, 0, 255); //-55 TOP

    //Motor Call for Centering on the XYZ HOME
    if(deltaMode == 0){
      xHome = 0;
      yHome = 0;
      zHome = 0;
    }
    if(deltaMode == 1){ //Throttle Mode
      xHome = 0;
      yHome = 0;
      zHome = zValue + Zshift;
    }
    if(deltaMode == 2){ //No Return
      xHome = xValue;
      yHome = yValue;
      zHome = zValue + Zshift;
    }

    deltaCal.InverseKinematic(xHome, yHome, zHome - Zshift, xT_HOME, yT_HOME, zT_HOME);

    DEBUG();
    checkDIR();
    submitUSB();
    delay(1);
}

void submitUSB() {
  if(lastUSB == USBDelay){
    //Delta Value USBdeadzone
    if((xMaster > 128 - USBdeadzone) && (xMaster < 128 + USBdeadzone))
    {
      xMaster = 128;
    }
    if((yMaster > 128 - USBdeadzone) && (yMaster < 128 + USBdeadzone))
    {
      yValue = 128;
    }
    if((zMaster > 128 - USBdeadzone) && (zMaster < 128 + USBdeadzone))
    {
      zMaster = 128;
    }

    //Thumb Value USBdeadzone
    if((xTValue > 512 - USBdeadzone) && (xTValue < 512 + USBdeadzone))
    {
      xTValue = 512;
    }
    if((yTValue > 512 - USBdeadzone) && (yTValue < 512 + USBdeadzone))
    {
      yTValue = 512;
    }
  
    //USB OUTPUT
      //Gamepad.press(0);
      if(mcp.digitalRead(0) == HIGH){
        Gamepad.press(14);
      }
      if(mcp.digitalRead(1) == HIGH){
        Gamepad.press(2);
      }
      if(mcp.digitalRead(2) == HIGH){
        Gamepad.press(3);
      }
      if(mcp.digitalRead(3) == HIGH){
        Gamepad.press(4);
      }
      if(mcp.digitalRead(4) == HIGH){
        Gamepad.press(1);
      }
      if(mcp.digitalRead(5) == HIGH){
        Gamepad.press(8);
      }
      if(mcp.digitalRead(6) == HIGH){
        Gamepad.press(10);
      }
      if(mcp.digitalRead(7) == HIGH){
        Gamepad.press(9);
      }
      if(mcp.digitalRead(8) == HIGH){
        Gamepad.press(12);
      }
      if(mcp.digitalRead(10) == HIGH){
        Gamepad.press(11);
      }
      if(mcp.digitalRead(12) == HIGH){
        Gamepad.press(5);
      }
      if(mcp.digitalRead(13) == HIGH){
        Gamepad.press(6);
      }
      if(mcp.digitalRead(14) == HIGH){
        Gamepad.press(7);
      }
      if(mcp.digitalRead(15) == HIGH){
        Gamepad.press(13);
      }
      
      Gamepad.xAxis(map(xMaster, 255, 0, -32765, 32765));
      Gamepad.yAxis(map(yMaster, 0, 255, -32765, 32765));
      Gamepad.rxAxis(map(zMaster, 0, 255, -32765, 32765));
      Gamepad.zAxis(map(xTValue, 0+xTCorrect, 1023+xTCorrect, -128, 127));
      Gamepad.rzAxis(map(yTValue, 0+yTCorrect, 1023+yTCorrect, -128, 127));
      Gamepad.dPad1(0);
      Gamepad.write();
      Gamepad.releaseAll();
      
    lastUSB = 0;
  }else{
    lastUSB++;
  }
}

void checkDIR() {
  if(xTheta-TZero+3 > xT_HOME + (deadzone*2)){
    digitalWrite(1, LOW);
    digitalWrite(0, HIGH);
    analogWrite(10, map(xTheta-TZero, 0, 75, 160, 80)); //180 / 105
  }else if(xTheta-TZero+3 < xT_HOME - (deadzone*2)){
    digitalWrite(1, HIGH);
    digitalWrite(0, LOW);
    analogWrite(10, map(xTheta-TZero, 0, -66, 160, 80));
  }else{
    digitalWrite(1, LOW);
    digitalWrite(0, LOW);
  }
  
  if(yTheta-TZero > yT_HOME + deadzone){
    digitalWrite(14, LOW);
    digitalWrite(15, HIGH);
    analogWrite(9, map(yTheta-TZero, 0, 75, 160, 80));
  }else if(yTheta-TZero < yT_HOME - deadzone){
    digitalWrite(14, HIGH);
    digitalWrite(15, LOW);
    analogWrite(9, map(yTheta-TZero, 0, -66, 160, 80)); 
  }else{
    digitalWrite(14, LOW);
    digitalWrite(15, LOW);
  }

  //analogWrite(6, returnPower);
  if(zTheta-TZero > zT_HOME + deadzone){
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    analogWrite(5, map(zTheta-TZero, 0, 75, 160, 80));
  }else if(zTheta-TZero < zT_HOME - deadzone){
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    analogWrite(5, map(zTheta-TZero, 0, -66, 160, 80));
  }else{
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }
}

void DEBUG () {
  if(lastDEBUG == DEBUGDelay && Debug == true){
    Serial.print("deltaMode: ");
    Serial.println(deltaMode);
    Serial.println("MASTER - RAW - DELTA - RETURN - MIN - MAX - POWER: ");
    Serial.print("X: ");
    Serial.print(xMaster);
    Serial.print(" - ");
    Serial.print(xTheta);
    Serial.print(" - ");
    Serial.print(xValue);
    Serial.print(" - ");
    Serial.print(xT_HOME + TZero);
    Serial.print(" - ");
    Serial.print(xMin);
    Serial.print(" - ");
    Serial.println(xMax);
    Serial.print("Y: ");
    Serial.print(yMaster);
    Serial.print(" - ");
    Serial.print(yTheta);
    Serial.print(" - ");
    Serial.print(yValue);
    Serial.print(" - ");
    Serial.print(yT_HOME + TZero);
    Serial.print(" - ");
    Serial.print(yMin);
    Serial.print(" - ");
    Serial.println(yMax);
    Serial.print("Z: ");
    Serial.print(zMaster);
    Serial.print(" - ");
    Serial.print(zTheta);
    Serial.print(" - ");
    Serial.print(zValue + Zshift);
    Serial.print(" - ");
    Serial.print(zT_HOME + TZero);
    Serial.print(" - ");
    Serial.print(zMin);
    Serial.print(" - ");
    Serial.println(zMax);

    if(xValue < xMin){ xMin = xValue; }
    if(yValue < yMin){ yMin = yValue; }
    if(zValue + Zshift < zMin){ zMin = zValue + Zshift; }

    if(xValue > xMax){ xMax = xValue; }
    if(yValue > yMax){ yMax = yValue; }
    if(zValue + Zshift > zMax){ zMax = zValue + Zshift; }
    
    lastDEBUG = 0;
  }else{
    lastDEBUG++;
  }
}
