#define LMF_PIN 9
#define LMR_PIN 10
#define RMF_PIN PD5
#define RMR_PIN PD6

#define LE1_PIN 12
#define LE2_PIN 13
#define RE1_PIN 1
#define RE2_PIN 2

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
  attachInterrupt(LE1_PIN, onLeftTick, FALLING);
  attachInterrupt(RE1_PIN, onRightTick, FALLING);
  Serial.begin(9600);
  
}

void loop() {
  motorLeft(0.5);
  motorRight(0.5);
  delay(500);
  motorLeft(0);
  motorRight(0);
  delay(500);
  motorLeft(-0.5);
  motorRight(-0.5);
  delay(500);
  motorLeft(0);
  motorRight(0);
  delay(500);
  Serial.println(leftTicks, rightTicks);
}

void onLeftTick() {
  if (digitalRead(LE2_PIN) == 0) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void onRightTick() {
  if (digitalRead(RE2_PIN) == 0) {
    rightTicks++;
  } else {
    rightTicks--;
  }
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
