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
#define BASIC_SPEED 200

unsigned long leftWheel = 0;		// pules come from encodes
unsigned long rightWheel = 0;
int KP = 0, KI = 0, KD = 0;
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
      Serial.println("Waiting for linking...");
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
  Serial3.write(sendbytes , 4);

  if (Serial3.available()) {
    Serial3.readBytes(receive, 4);
    nextPoint_X = receive[0] * 256 + receive[1];
    nextPoint_Y = receive[2] * 256 + receive[3];
  }

  hea = atan2(nextPoint_X - currentPoint_X, nextPoint_Y - currentPoint_Y) * 180 / PI;
  if (hea < 0) hea += 360; // assigned_heading: [0, 360)

  dis = sqrt((nextPoint_X - currentPoint_X)*(nextPoint_X - currentPoint_X) + (nextPoint_Y - currentPoint_Y)*(nextPoint_Y - currentPoint_Y));

  turnAround(hea);
  goStraight(dis);
}

void turnAround(int assigned_heading) {
  byte Dir;
  int Output = 0;
  int e[3] = {0, 0, 0};
  compass.read();
  int heading = compass.heading();
  int Input = assigned_heading - heading;
  
  while (abs(Input) > 2) {
    e[2] = e[1];
    e[1] = e[0];
    e[0] = Input;
    Output = KP * (e[0] - e[1]) + KI * (e[0]) - KD * (e[0] - 2 * e[1] + e[2]) + Output;
    if (Output > 0)
    {
      Dir = HIGH;
    } else {
      Dir = LOW;
    }

    digitalWrite(4, Dir);
    digitalWrite(7, !Dir);
    analogWrite(MOTORL_PIN, constrain(MIN_TURN_W, 0, 255));
    analogWrite(MOTORR_PIN, constrain(MIN_TURN_W, 0, 255));

    compass.read();
    heading = compass.heading();
    Input = assigned_heading - heading;
  }
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
    sDis = (rightWheel + leftWheel) / 2 / 333 * 22.0;
    if (rightWheel - leftWheel > 4)
    { bspeedR--;
      analogWrite(MOTORR_PIN, constrain(BASIC_SPEED + bspeedR * 5, 0, 255));
    } else if (rightWheel - leftWheel < -4)
    { bspeedL--;
      analogWrite(MOTORL_PIN, constrain(BASIC_SPEED + bspeedL * 5, 0, 255));
    } else {
      bspeedL = 0;
      bspeedR = 0;
    }
    if (sDis >= distance) break;
  }
}

void count1() {
  rightWheel++;
}
void count2() {
  leftWheel++;
}
