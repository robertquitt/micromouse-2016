#include <NewPing.h>

#define LMF_PIN 9
#define LMR_PIN 10
#define RMF_PIN 5
#define RMR_PIN 6

#define LE1_PIN 3
#define LE2_PIN 8
#define RE1_PIN 2   
#define RE2_PIN 7

#define TRIGGER_PIN 11
#define ECHO_PIN 12

#define Apin 14  // Analog battery monitoring pin

#define NUMREADINGS 10
#define LOOPTIME 100

#define MAX_DISTANCE 200
#define LOWPASS 0.5

const float Kp = 3.0;
const float Ki = 0.05;
const float Kd = 0.1;
int readings[NUMREADINGS];
volatile long leftTicks = 0;
volatile long rightTicks = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
float uS = 0;

void setup() {
  pinMode(LMF_PIN, OUTPUT);
  pinMode(LMR_PIN, OUTPUT);
  pinMode(RMF_PIN, OUTPUT);
  pinMode(RMR_PIN, OUTPUT);
  pinMode(LE1_PIN, INPUT);
  pinMode(LE2_PIN, INPUT);
  pinMode(RE1_PIN, INPUT);
  pinMode(RE2_PIN, INPUT);
//  for(int i=0; i<NUMREADINGS; i++)
//    readings[i] = 0;
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LE1_PIN), onLeftTick, FALLING);
  attachInterrupt(digitalPinToInterrupt(RE1_PIN), onRightTick, FALLING);
  Serial.println("Program Begin");
}

unsigned long lastMilli = 0;
int lastLeftError = 0;
int lastRightError = 0;
int target = 300;
long leftI = 0;
long rightI = 0;

int l, r;

void loop() {
  if (millis() - lastMilli >= LOOPTIME) {
    l = updateLeft(0, 500, leftTicks, millis()-lastMilli);
    motorLeft(l);

    r = updateRight(0, 500, rightTicks, millis()-lastMilli);
    motorRight(r);
    
    lastMilli = millis();
  }
  
//  if (rightTicks < 300) {
//    motorRight(127);
//  } else if (rightTicks > 310){
//    motorRight(-127);
//  } else {
//    motorRight(0);
//  }
//  if (leftTicks < 300) {
//    motorLeft(127);
//  } else if (leftTicks > 310){
//    motorLeft(-127);
//  } else {
//    motorLeft(0);
//  }


  //SONAR CODE BELOW
  // raw sensor data
   uS = sonar.ping_mm();
   
 //uncomment to implement lowpass filtering
   float usPrev = uS;
   uS = sonar.ping_mm() * (1-LOWPASS) + usPrev * LOWPASS;

   Serial.print("Sonar reading:");
   Serial.println(uS);
   delay(100);

}

int updateLeft(int command, int targetValue, int currentValue, int elapsed) {
  int leftError = targetValue - currentValue;
  leftI += leftError;
  float pidTerm = (Kp * leftError) + (Kd * (leftError - lastLeftError)) + (Ki * leftI);
  int out = constrain(command+int(pidTerm), -255, 255);
  /*Serial.print("Left ticks: ");
  Serial.print(currentValue);
  Serial.print(" elapsed: ");
  Serial.print(elapsed);
  Serial.print(" P: ");
  Serial.print(leftError);
  Serial.print(" I: ");
  Serial.print(leftI);
  Serial.print(" D: ");
  Serial.print(leftError - lastLeftError);
  Serial.print(" OUT: ");
  Serial.println(out);*/
  lastLeftError = leftError;
  if (abs(out) < 40) {
    return 0;
  } else {
    return out;
  }
}

int updateRight(int command, int targetValue, int currentValue, int elapsed) {
  int rightError = targetValue - currentValue;
  rightI += rightError;
  float pidTerm = (Kp * rightError) + (Kd * (rightError - lastRightError)) +  (Ki * rightI);

  int out = constrain(command+int(pidTerm), -255, 255);
  /*Serial.print("Right ticks: ");
  Serial.print(currentValue);
  Serial.print(" elapsed: ");
  Serial.print(elapsed);
  Serial.print(" P: ");
  Serial.print(rightError);
  Serial.print(" I: ");
  Serial.print(rightI);
  Serial.print(" D: ");
  Serial.print(rightError - lastRightError);
  Serial.print(" OUT: ");
  Serial.println(out);*/
  lastRightError = rightError;
  if (abs(out) < 40) {
    return 0;
  } else {
    return out;
  }
}

void onLeftTick() {
  if (digitalRead(LE2_PIN) == LOW) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void onRightTick() {
  if (digitalRead(RE2_PIN) == LOW ) {
    rightTicks--;
  } else {
    rightTicks++;
  }
}

void motorLeft(int spd) {
  if (spd > 0) {
    analogWrite(LMF_PIN, spd);
    analogWrite(LMR_PIN, 0);
  } else if (spd < 0) {
    analogWrite(LMF_PIN, 0);
    analogWrite(LMR_PIN, -spd);
  } else {
    analogWrite(LMF_PIN, 0);
    analogWrite(LMR_PIN, 0);
  }
}

void motorRight(int spd) {
  if (spd > 0) {
    analogWrite(RMF_PIN, spd);
    analogWrite(RMR_PIN, 0);
  } else if (spd < 0) {
    analogWrite(RMF_PIN, 0);
    analogWrite(RMR_PIN, -spd);
  } else {
    analogWrite(RMF_PIN, 0);
    analogWrite(RMR_PIN, 0);
  }
}
