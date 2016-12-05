#define LMF_PIN 7
#define LMR_PIN 8
#define LMPWM_PIN 6
#define RMF_PIN 10
#define RMR_PIN 11
#define RMPWM_PIN 9

#define LE1_PIN 3
#define LE2_PIN 5
#define RE1_PIN 2
#define RE2_PIN 4

#define Apin 14  // Analog battery monitoring pin

#define NUMREADINGS 10
#define LOOPTIME 100

const float Kp = 0.4;
const float Kd = 1;
const float Ki = 0.1;
int readings[NUMREADINGS];
volatile long leftTicks = 0;
volatile long rightTicks = 0;



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
}

unsigned long lastMilli = 0;
int lastLeftError = 0;
int lastRightError = 0;
int target = 300;
int leftI = 0;
int rightI = 0;

void loop() {
  if (millis() - lastMilli >= LOOPTIME) {
    motorLeft(updateLeft(0, 500, leftTicks));
    motorRight(updateRight(0, 500, rightTicks));
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
}

int updateLeft(int command, int targetValue, int currentValue) {
  int leftError = targetValue - currentValue;
  leftI += leftError - lastLeftError;
  float pidTerm = (Kp * leftError) + (Kd * (leftError - lastLeftError)) + (Ki * leftI);
  lastLeftError = leftError;
  int out = constrain(command+int(pidTerm), -256, 255);
  if (abs(out) < 40) {
    return 0;
  } else {
    return out;
  }
}

int updateRight(int command, int targetValue, int currentValue) {
  int rightError = targetValue - currentValue;
  rightI += rightError - lastRightError;
  float pidTerm = (Kp * rightError) + (Kd * (rightError - lastRightError)) +  (Ki * rightI);
  lastRightError = rightError;
  int out = constrain(command+int(pidTerm), -255, 255);
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
