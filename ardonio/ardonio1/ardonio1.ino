#include <Wire.h>
#include <LSM303.h>
#include<NewPing.h>
#include <math.h>

#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(41, 42, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(43, 44, MAX_DISTANCE),
  NewPing(45, 46, MAX_DISTANCE)
};

#define INT_PIN 2
#define INT_PIN2 3
#define MOTORL_PIN 5
#define MOTORR_PIN 6

#define MIN_TURN_W  180 // min output gonglv when turning around
#define BASIC_SPEED 250

long leftWheel = 0;		// pules come from encodes
long rightWheel = 0;
float KP = 3, KI = 0.0005, KD = 0;
byte receive[10], sendbytes[10];
int currentPoint_X, currentPoint_Y, nextPoint_X, nextPoint_Y;
int northAt;


LSM303 compass;

void setup()
{
  pinMode(INT_PIN, INPUT_PULLUP);
  pinMode(INT_PIN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), count1, RISING);
  attachInterrupt(digitalPinToInterrupt(INT_PIN2), count2, RISING);
  for (int i = 4; i <= 7; i++) //Pin 4 to 7 are used
    pinMode(i, OUTPUT);
  Serial.begin(9600);
  Serial3.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  bool handshake = false;
  while (!handshake)
  {
    if (Serial3.available())
    {
      int inByte = Serial3.read();
      if (inByte == 18)
      {
        Serial3.print(11, HEX);
        Serial.println("Hello");
      }
      else if (inByte == 17)
      {
        Serial.println("ACK");
        handshake = true;//握手完成
      }
    }
    else
    {
      //Serial.println("Waiting for linking...");
    }
  }

  bool readinit = false;
  while (!readinit)
  {
    if (Serial3.available())
    {
      Serial.println("Connection is done!");
      Serial3.readBytes(receive, 6);
      currentPoint_X = receive[0] * 256 + receive[1];
      currentPoint_Y = receive[2] * 256 + receive[3];
      northAt = receive[4] * 256 + receive[5];
      readinit = true;//初始化数据接受完成
      Serial.print("initial x=");
      Serial.print(currentPoint_X);
      Serial.print("initial y=");
      Serial.println(currentPoint_Y);

      Serial3.print(255, HEX);
      Serial.println("Initial data has recieved");
    }
    else
    {
      Serial.println("Waiting for initial data");
    }
  }
}


void loop()
{
  int hea;
  double dis;
  compass.read();
  int heading = compass.heading();
  sendbytes[0] = byte(currentPoint_X / 256);
  sendbytes[1] = byte(currentPoint_X % 256);
  sendbytes[2] = byte(currentPoint_Y / 256);
  sendbytes[3] = byte(currentPoint_Y % 256);
  sendbytes[4] = byte(heading / 256);
  sendbytes[5] = byte(heading % 256);
  Serial3.write(sendbytes , 6);

  int isReceive = 0;
  while (isReceive == 0) {
    if (Serial3.available()) {
      Serial3.readBytes(receive, 4);
      nextPoint_X = receive[0] * 256 + receive[1];
      nextPoint_Y = receive[2] * 256 + receive[3];

      Serial.print("current x,y and next x,y:");
      Serial.print(currentPoint_X);
      Serial.print(" ");
      Serial.print(currentPoint_Y);
      Serial.print(", ");
      Serial.print(nextPoint_X);
      Serial.print(" ");
      Serial.println(nextPoint_Y);
      Stop(2);
      
      hea = atan2(nextPoint_X - currentPoint_X, nextPoint_Y - currentPoint_Y) * 180 / PI;
      if (hea < 0) hea += 360; // assigned_heading: [0, 360)

      if (nextPoint_X == currentPoint_X) dis = abs(nextPoint_Y - currentPoint_Y);
      else dis = abs(nextPoint_X - currentPoint_X);
//      dis = sqrt((nextPoint_X - currentPoint_X) * (nextPoint_X - currentPoint_X) + (nextPoint_Y - currentPoint_Y) * (nextPoint_Y - currentPoint_Y));

      Serial.print("turn around at angle");
      Serial.println(hea);
      turnAround(hea);
      goStraight1(dis);
      Stop(2);
      
      isReceive = 1;
    }
  }
}

void turnAround(int assigned_heading) {
  //  assigned_heading += northAt;
  //  if (assigned_heading > 360) assigned_heading -= 360;

  compass.read();
  float heading1 = compass.heading() + northAt;
  if (heading1 > 360) heading1 -= 360;

  byte Dir;

  while (abs(assigned_heading - heading1) > 3) {
    if (assigned_heading - heading1 > 0)
    {
      Dir = HIGH;
    } else {
      Dir = LOW;
    }
    Serial.print("AH:");
    Serial.print(assigned_heading);
    Serial.print(" H:");
    Serial.print(heading1);
    Serial.print("  Dir:");
    Serial.println(Dir);
    //    digitalWrite(4, Dir);
    //    digitalWrite(7, !Dir);
    //analogWrite(MOTORL_PIN, constrain(MIN_TURN_W+3*abs(assigned_heading - heading1), 0, 255));
    //analogWrite(MOTORR_PIN, constrain(MIN_TURN_W+3*abs(assigned_heading - heading1), 0, 255));
    digitalWrite(4, 1);
    digitalWrite(7, 1);
    if (Dir)
    {
      analogWrite(MOTORL_PIN, constrain(MIN_TURN_W + 3 * abs(assigned_heading - heading1), 0, 255));
      analogWrite(MOTORR_PIN, 0);
    }
    else
    {
      analogWrite(MOTORR_PIN, constrain(MIN_TURN_W + 3 * abs(assigned_heading - heading1), 0, 255));
      analogWrite(MOTORL_PIN, 0);
    }


    compass.read();
    heading1 = compass.heading() + northAt;
    if (heading1 > 360) heading1 -= 360;

  }
  Serial.print("turn around success! now heading:");
  Serial.println(heading1);
}

void goStraight(double distance) {
  double sDis = 0;

  digitalWrite(4, 1);
  digitalWrite(7, 1);
  int bspeedL = 0; // adjust l wheel speed
  int bspeedR = 0;

  analogWrite(MOTORL_PIN, constrain(BASIC_SPEED, 0, 255));
  analogWrite(MOTORR_PIN, constrain(BASIC_SPEED, 0, 255));

  rightWheel = 0;
  leftWheel = 0;

  while (true) {
    sDis = (rightWheel + leftWheel) / 2 / 333 * 22.0 /190*250;
    if (rightWheel - leftWheel > 4)
    { bspeedR--;
      analogWrite(MOTORL_PIN, constrain(BASIC_SPEED + bspeedR * 5, 0, 255));
    } else if (rightWheel - leftWheel < -4)
    { bspeedL--;
      analogWrite(MOTORR_PIN, constrain(BASIC_SPEED + bspeedL * 5, 0, 255));
    } else {
      bspeedL = 0;
      bspeedR = 0;
    }
    if (sDis >= distance) break;

    Serial.print("distance");
    Serial.print(distance);
    Serial.print(" sDis:");
    Serial.print(sDis);
    Serial.print(" right-left chazhi:");
    Serial.println(rightWheel - leftWheel);
  }
}

void goStraight1(double distance) {
  double sDis = 0;
  rightWheel = 0;
  leftWheel = 0;

  digitalWrite(4, 1);
  digitalWrite(7, 1);
  int bspeedL = 0; // adjust l wheel speed
  int bspeedR = 0;
  float e[3] = {0, 0, 0};
  float PID_Output = 0;
  while (true)
  {
    sDis = (rightWheel + leftWheel) / 2 / 8.9;
    int wheelErro = -leftWheel + rightWheel;
    e[2] = e[1];
    e[1] = e[0];
    e[0] = wheelErro; //Input;
    PID_Output = KP * (e[0] - e[1]) + KI * (e[0]) - KD * (e[0] - 2 * e[1] + e[2]) + PID_Output;

    analogWrite(MOTORL_PIN, constrain(250 + int(PID_Output), 0, 255));
    analogWrite(MOTORR_PIN, constrain(250 - int(PID_Output), 0, 255));

    if (sDis >= distance) break;
    
    Serial.print("distance: ");
    Serial.print(distance);
    Serial.print(" sDis:");
    Serial.print(sDis);
    Serial.print(" right-left chazhi:");
    Serial.println(rightWheel - leftWheel);
  }
}

void Stop(int i) {
  delay(i * 1000);
}
void count1() {
  rightWheel++;
}
void count2() {
  leftWheel++;
}
