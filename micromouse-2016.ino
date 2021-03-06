#include <Wire.h>

#include <SparkFun_VL6180X.h>

#include <NewPing.h>

#define LMF_PIN 8
#define LMR_PIN 7
#define LMPWM_PIN 6
#define RMF_PIN A0
#define RMR_PIN A1
#define RMPWM_PIN 9

#define LE1_PIN 3
#define LE2_PIN 5
#define RE1_PIN 2
#define RE2_PIN 4

#define LTRIG_PIN 11
#define LECHO_PIN 12

#define RTRIG_PIN A2
#define RECHO_PIN A3

#define TOF_SDA_PIN A4
#define TOF_SCC_PIN A5

#define NUMREADINGS 10
#define LOOPTIME 100

#define MAX_DISTANCE 200
#define LOWPASS 0.5

#define SLOW 1
#define THRESHOLD 5

#define VL6180X_ADDRESS 0x29

const float Kpl = 10;
const float Kdl = 5;
const float Kil = 0;

const float Kpr = 10;
const float Kdr = 5;
const float Kir = 0;

int rightI[NUMREADINGS];
int leftI[NUMREADINGS];

int lCommand = 0;
int rCommand = 0;

short r_i = 0;
short l_i = 0;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);

NewPing leftSonar(LTRIG_PIN, LECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing rightSonar(RTRIG_PIN, RECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

float uS_L = 0;
float uS_R = 0;
float frontRange = 0;

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
  Wire.begin(); //Start I2C library
  delay(100); // delay .1s

    sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
  printIdentification(&identification); // Helper function to print all the Module information

    if(sensor.VL6180xInit() != 0){
    Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
  };

  sensor.VL6180xDefautSettings(); //Load default settings to get started.

    delay(1000); // delay 1s

  attachInterrupt(digitalPinToInterrupt(LE1_PIN), onLeftTick, FALLING);
  attachInterrupt(digitalPinToInterrupt(RE1_PIN), onRightTick, FALLING);
//  Serial.println("Program Begin");
}

unsigned long lastMilli = 0;
int lastLeftError = 0;
int lastRightError = 0;
int lTarget = 0;
int rTarget = 0;

int l, r;

void loop() {
  //IR SENSOR CODE BELOW

  //Get Ambient Light level and report in LUX
//  Serial.print("Ambient Light Level (Lux) = ");

  //Input GAIN for light levels,
  // GAIN_20     // Actual ALS Gain of 20
  // GAIN_10     // Actual ALS Gain of 10.32
  // GAIN_5      // Actual ALS Gain of 5.21
  // GAIN_2_5    // Actual ALS Gain of 2.60
  // GAIN_1_67   // Actual ALS Gain of 1.72
  // GAIN_1_25   // Actual ALS Gain of 1.28
  // GAIN_1      // Actual ALS Gain of 1.01
  // GAIN_40     // Actual ALS Gain of 40

//  Serial.println( sensor.getAmbientLight(GAIN_1) );

  //Get Distance and report in mm
//  Serial.print("Distance measured (mm) = ");
  frontRange = sensor.getDistance();
  Serial.print( frontRange );
  Serial.print(" ");

  delay(100);

  //SONAR CODE BELOW
  // raw sensor data
   uS_L = leftSonar.ping_mm();
   uS_R = rightSonar.ping_mm();

   float us_LPrev = uS_L;
   uS_L = leftSonar.ping_mm() * (1-LOWPASS) + us_LPrev * LOWPASS;

   float us_RPrev = uS_R;
   uS_R = rightSonar.ping_mm() * (1-LOWPASS) + us_RPrev * LOWPASS;

//   Serial.print("Left sonar reading:");
   Serial.print(uS_L);
   Serial.print(" ");
//   Serial.print("Right sonar reading:");
   Serial.print(uS_R);
   Serial.println();
   delay(100);

   //MOTOR CODE BELOW
   if (abs(leftTicks - lTarget) < 20 && (leftTicks - lTarget) < 20){
      updateTarget(uS_L, uS_R, frontRange);    
   }
  if (millis() - lastMilli >= LOOPTIME) {
    l = updateLeft(0, lTarget, leftTicks, millis()-lastMilli);
    motorLeft(l);

    r = updateRight(0, rTarget, rightTicks, millis()-lastMilli);
    motorRight(r);

    lastMilli = millis();
  }
}

int updateTarget(float uS_L, float uS_R, float frontRange) {
  
  if (uS_R > 100) {
    lTarget = leftTicks + 15;
    rTarget = rightTicks - 15;
  } else if (frontRange < 100){  
    lTarget = leftTicks;
    rTarget = rightTicks; 
  } else if (frontRange < 100 && uS_R < 100){
    lTarget = leftTicks - 15;
    rTarget = rightTicks + 15; 
  } else {
    lTarget += 40;
    rTarget += 40;
  }
}

void updateCommand(int leftCommand, int rightCommand, float leftSensor, float rightSensor) {
  if (leftSensor < 20) {
    rightCommand = -30;
  } else if (rightSensor < 20) {
    leftCommand = -30;
  }
  if (frontRange < 100) {
    lTarget = leftTicks;
    rTarget = rightTicks;
  }
}

int updateLeft(int command, int targetValue, int currentValue, int elapsed) {
  int leftError = targetValue - currentValue;
  int sum = 0;
  for (short i=0; i<NUMREADINGS; i++) {
    if (i == l_i) {
      leftI[i] = leftError;
    }
    sum += leftI[i];
  }
  l_i++;
  if(l_i == NUMREADINGS) {
    l_i = 0;
  }
  float pidTerm = (Kpl * leftError) + (Kdl * (leftError - lastLeftError)) + (Kil * sum);
  int out = constrain(command+int(pidTerm), -255, 255);
  Serial.print("Left ticks: ");
  Serial.print(currentValue);
  Serial.print(" elapsed: ");
  Serial.print(elapsed);
  Serial.print(" P: ");
  Serial.print(leftError);
  Serial.print(" I: ");
  Serial.print(sum); 
  Serial.print(" D: ");
  Serial.print(leftError - lastLeftError);
  Serial.print(" OUT: ");
  Serial.println(out);
  lastLeftError = leftError;
  if (abs(out) < THRESHOLD) {
    return 0;
  } else {
    return out;
  }
}

int updateRight(int command, int targetValue, int currentValue, int elapsed) {
  int rightError = targetValue - currentValue;
  int sum = 0;
  for (short i=0; i<NUMREADINGS; i++) {
    if (i == r_i) {
      rightI[i] = rightError;
    }
    sum += rightI[i];
  }
  r_i++;
  if(r_i == NUMREADINGS) {
    r_i = 0;
  }
  float pidTerm = (Kpr * rightError) + (Kdr * (rightError - lastRightError)) + (Kir * sum);

  int out = constrain(command+int(pidTerm), -255, 255);
//  Serial.print("Right ticks: ");
//  Serial.print(currentValue);
//  Serial.print(" elapsed: ");
//  Serial.print(elapsed);
//  Serial.print(" P: ");
//  Serial.print(rightError);
//  Serial.print(" D: ");
//  Serial.print(rightError - lastRightError);
//  Serial.print(" OUT: ");
//  Serial.println(out);
  lastRightError = rightError;
  if (abs(out) < THRESHOLD) {
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

void motorLeft(int spd, bool reversed) {
  if (spd > 0) {
    analogWrite(LMPWM_PIN, spd);
    if (reversed) {
      digitalWrite(LMF_PIN, HIGH);
      digitalWrite(LMR_PIN, LOW);
    } else {
      digitalWrite(LMF_PIN, LOW);
      digitalWrite(LMR_PIN, HIGH);
    }
  } else {
    analogWrite(LMPWM_PIN, 0);
    digitalWrite(LMF_PIN, LOW);
    digitalWrite(LMR_PIN, LOW);
  }
}

void motorLeft(int spd) {
  if (spd > 0) {
    analogWrite(LMPWM_PIN, spd);
    digitalWrite(LMF_PIN, HIGH);
    digitalWrite(LMR_PIN, LOW);
  } else if (spd < 0) {
    analogWrite(LMPWM_PIN, -spd);
    digitalWrite(LMF_PIN, LOW);
    digitalWrite(LMR_PIN, HIGH);
  } else {
    analogWrite(LMPWM_PIN, 0);
    digitalWrite(LMF_PIN, LOW);
    digitalWrite(LMR_PIN, LOW);
  }
}

void motorRight(int spd, bool reversed) {
  if (spd > 0) {
    analogWrite(RMPWM_PIN, spd);
    if (reversed) {
      digitalWrite(RMF_PIN, HIGH);
      digitalWrite(RMR_PIN, LOW);
    } else {
      digitalWrite(RMF_PIN, LOW);
      digitalWrite(RMR_PIN, HIGH);
    }
  } else {
    analogWrite(RMPWM_PIN, 0);
    digitalWrite(RMF_PIN, LOW);
    digitalWrite(RMR_PIN, LOW);
  }
}

void motorRight(int spd) {
  if (spd > 0) {
    analogWrite(RMPWM_PIN, spd);
    digitalWrite(RMF_PIN, HIGH);
    digitalWrite(RMR_PIN, LOW);
  } else if (spd < 0) {
    analogWrite(RMPWM_PIN, -spd);
    digitalWrite(RMF_PIN, LOW);
    digitalWrite(RMR_PIN, HIGH);
  } else {
    analogWrite(RMPWM_PIN, 0);
    digitalWrite(RMF_PIN, LOW);
    digitalWrite(RMR_PIN, LOW);
  }
}

void printIdentification(struct VL6180xIdentification *temp){
 /* Serial.print("Model ID = ");
  Serial.println(temp->idModel);

  Serial.print("Model Rev = ");
  Serial.print(temp->idModelRevMajor);
  Serial.print(".");
  Serial.println(temp->idModelRevMinor);

  Serial.print("Module Rev = ");
  Serial.print(temp->idModuleRevMajor);
  Serial.print(".");
  Serial.println(temp->idModuleRevMinor);

  Serial.print("Manufacture Date = ");
  Serial.print((temp->idDate >> 3) & 0x001F);
  Serial.print("/");
  Serial.print((temp->idDate >> 8) & 0x000F);
  Serial.print("/1");
  Serial.print((temp->idDate >> 12) & 0x000F);
  Serial.print(" Phase: ");
  Serial.println(temp->idDate & 0x0007);

  Serial.print("Manufacture Time (s)= ");
  Serial.println(temp->idTime * 2);
  Serial.println();
  Serial.println();*/
}
