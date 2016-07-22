#include <Wire.h>
#include <LSM303.h>
#include <NewPing.h>
#include <math.h>

#define MOTORL_PIN 5
#define MOTORR_PIN 6
#define TRIGGER_PIN  41  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     42  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define LED_PIN     13
const byte interruptPin = 2;
const byte interruptPin2 = 3;

#define KP 3
#define KI 0
#define KD 0
#define dead_zone 5 //compass's dead zone

float e[3] = {0, 0, 0};

long sL = 0;
long sR = 0;

long LoopTime, LastLoop, LoopCount;
LSM303 compass;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

float nowX = 0;
float nowY = 0;
float nowDistance = 0;
float distance = 0;

int assigned_heading = 0;
int heading;
int obst;
int turnSpeed;

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink1, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), blink2, RISING);

  for (int i = 4; i <= 7; i++) //Pin 4 to 7 are used
    pinMode(i, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  initial();
}

void loop() {
  obst = sonar.ping_cm();

  compass.read();
  heading = compass.heading();

  controller();
  print();

  stop();
}

void blink1() {
  sL++;
}
void blink2() {
  sR++;
}

// calculate the current x,y
void calVertex() {
  nowX = nowX + sin(heading / 180 * 3.1415) * (distance - nowDistance);
  nowY = nowY + cos(heading / 180 * 3.1415) * (distance - nowDistance);
  nowDistance = distance;
}

// isForward: whether forward, if false, backward
void move(int isForward) {
  digitalWrite(4, isForward);
  digitalWrite(7, isForward);
  analogWrite(MOTORL_PIN, constrain(230 - turnSpeed, 0, 255));
  analogWrite(MOTORR_PIN, constrain(230 + turnSpeed, 0, 255));
}

void turnAround(int angle) {
  int signal;
  if (angle > 0)
    signal = 1;
  else signal  = 0;

  digitalWrite(4, !signal);
  digitalWrite(7, signal);
  analogWrite(MOTORL_PIN, 200);
  analogWrite(MOTORR_PIN, 200);
}

void initial() {
  assigned_heading = 0;
  while (true) {
    controller();

    compass.read();
    if (abs(assigned_heading - compass.heading()) < 5)
      break;
  }
}

// assigned_heading, distance
// decide what to do, turn left/right, walk forward/backward
void controller() {
  int turnAngle = assigned_heading - heading;
  if (turnAngle > 180) {
    turnAngle = turnAngle - 360;
  } else if (turnAngle < -180) {
    turnAngle = turnAngle + 360;
  }

  e[2] = e[1];
  e[1] = e[0];
  e[0] = turnAngle;
  turnSpeed += KP * (e[0] - e[1]) + KI * (e[0]) + KD * (e[0] - 2 * e[1] + e[2]) + turnSpeed;

  if (abs(turnAngle) < dead_zone)
  {
    move(1);
  } else {
    turnAround(turnAngle);
  }
}

void receiveData() {
  assigned_heading = 90;
}

void sendData() {

}

void stop() {
  char input = Serial.read();
  if (input == 'x') {
    analogWrite(MOTORL_PIN, 0);
    analogWrite(MOTORR_PIN, 0);
  }
}

void print() {
  LoopTime = millis() - LastLoop;
  LoopCount = LoopCount + LoopTime;
  if (LoopCount > 200) {
    Serial.print("H:");
    Serial.print(heading);
    Serial.print("assigned_heading: ");
    Serial.print(assigned_heading);
    double distance = sL * 22 / 48 / 4;
    double distance2 = sR * 22 / 48 / 4;
    Serial.print("  LD: ");
    Serial.print(distance);
    Serial.print("cm  RD:");
    Serial.print(distance2);
    Serial.print("cm  S0:");
    Serial.print(obst);
    Serial.println("cm");
    LoopCount = 0;
  }
  LastLoop = millis();
}
