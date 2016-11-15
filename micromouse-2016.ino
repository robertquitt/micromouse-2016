#define LMF_PIN 9
#define LMR_PIN 10
#define RMF_PIN 5
#define RMR_PIN 6

#define LE1_PIN 3
#define LE2_PIN 8
#define RE1_PIN 2
#define RE2_PIN 7

volatile int leftTicks = 0;
volatile int rightTicks = 0;



void setup() {
  pinMode(LMF_PIN, OUTPUT);
  pinMode(LMR_PIN, OUTPUT);
  pinMode(RMF_PIN, OUTPUT);
  pinMode(RMR_PIN, OUTPUT);
  pinMode(LE1_PIN, INPUT);
  pinMode(LE2_PIN, INPUT);
  pinMode(RE1_PIN, INPUT);
  pinMode(RE2_PIN, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LE1_PIN), onLeftTick, FALLING);
  attachInterrupt(digitalPinToInterrupt(RE1_PIN), onRightTick, FALLING);
}

void loop() {
  if (rightTicks < 300) {
    motorRight(0.1);
  } else if (rightTicks > 310){
    motorRight(-0.1);
  } else {
    motorRight(0);
  }
  if (leftTicks < 300) {
    motorLeft(0.15);
  } else if (leftTicks > 310){
    motorLeft(-0.15);
  } else {
    motorLeft(0);
  }
}

void onLeftTick() {
  if (digitalRead(LE2_PIN) == 0) {
    leftTicks++;
  } else {
    leftTicks--;
  }
  Serial.print("Left: ");
  Serial.println(leftTicks);
}

void onRightTick() {
  if (digitalRead(RE2_PIN) == 0) {
    rightTicks--;
  } else {
    rightTicks++;
  }
  Serial.print("Right: ");
  Serial.println(rightTicks);
}

void motorLeft(float spd) {
  if (spd > 0) {
    analogWrite(LMF_PIN, spd * 255);
    analogWrite(LMR_PIN, 0);
  } else if (spd < 0) {
    analogWrite(LMF_PIN, 0);
    analogWrite(LMR_PIN, -spd * 255);
  } else {
    analogWrite(LMF_PIN, 0);
    analogWrite(LMR_PIN, 0);
  }
}

void motorRight(float spd) {
  if (spd > 0) {
    analogWrite(RMF_PIN, spd * 255);
    analogWrite(RMR_PIN, 0);
  } else if (spd < 0) {
    analogWrite(RMF_PIN, 0);
    analogWrite(RMR_PIN, -spd * 255);
  } else {
    analogWrite(RMF_PIN, 0);
    analogWrite(RMR_PIN, 0);
  }
}
