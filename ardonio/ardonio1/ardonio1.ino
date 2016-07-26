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
float KP = 5, KI = 0.001, KD = 0;
byte receive[4], sendbytes[10], receive1[6];
int currentPoint_X, currentPoint_Y, nextPoint_X, nextPoint_Y;
int lastPoint_X = -1, lastPoint_Y = -1;
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

  initialVetex();
}

void initialVetex() {
  bool readinit = false;
  while (!readinit)
  {
    if (Serial3.available())
    {
      Serial.println("Connection is done!");
      Serial3.readBytes(receive1, 6);
      currentPoint_X = receive1[0] * 256 + receive1[1];
      currentPoint_Y = receive1[2] * 256 + receive1[3];
      northAt = receive1[4] * 256 + receive1[5];
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
int isReceive;

void loop()
{
  double hea;
  double dis;
  compass.read();
  int heading = compass.heading();

  Serial.print("send to pi current x: ");
  Serial.print(currentPoint_X);
  Serial.print(" send to pi current y: ");
  Serial.print(currentPoint_Y);
  Serial.println(", ");

  sendbytes[0] = byte(currentPoint_X / 256);
  sendbytes[1] = byte(currentPoint_X % 256);
  sendbytes[2] = byte(currentPoint_Y / 256);
  sendbytes[3] = byte(currentPoint_Y % 256);
  sendbytes[4] = byte(heading / 256);
  sendbytes[5] = byte(heading % 256);
  Serial3.write(sendbytes , 6);

  int isMapSwitch = 0;
  isReceive = 0;
  while (isReceive == 0) {
    if (Serial3.available()) {
      Serial3.readBytes(receive, 5);
      isMapSwitch = receive[0];
      nextPoint_X = receive[1] * 256 + receive[2];
      nextPoint_Y = receive[3] * 256 + receive[4];

      if (isMapSwitch == 1) {
        initialVetex();
      } else {

        Serial.print("last x,y, current x,y and next x,y:");
        Serial.print(lastPoint_X);
        Serial.print(" ");
        Serial.print(lastPoint_Y);
        Serial.print(", ");
        Serial.print(currentPoint_X);
        Serial.print(" ");
        Serial.print(currentPoint_Y);
        Serial.print(", ");
        Serial.print(nextPoint_X);
        Serial.print(" ");
        Serial.println(nextPoint_Y);
        //Stop(2);
        if ((currentPoint_X == nextPoint_X) && (nextPoint_Y == currentPoint_Y)) break;

        hea = atan2(nextPoint_X - currentPoint_X, nextPoint_Y - currentPoint_Y) * 180 / PI;
        if (hea < 0) hea += 360; // assigned_heading: [0, 360)

        //if (nextPoint_X == currentPoint_X) dis = abs(nextPoint_Y - currentPoint_Y);
        //else dis = abs(nextPoint_X - currentPoint_X);
        dis = sqrt(sq(float(nextPoint_X - currentPoint_X)) + sq(float(nextPoint_Y - currentPoint_Y)));

        Serial.print("turn around at angle:　");
        Serial.println(hea);
        //      turnAround1();
        turnAround1();
        goStraight(dis);
        Stop(2);
        lastPoint_X = currentPoint_X;
        lastPoint_Y = currentPoint_Y;
        currentPoint_X = nextPoint_X;
        currentPoint_Y = nextPoint_Y;
      }
      isReceive = 1;
    }
  }
}

//void turnAround(int assigned_heading) {
//  //  assigned_heading += northAt;
//  //  if (assigned_heading > 360) assigned_heading -= 360;
//
//  compass.read();
//  float heading1 = compass.heading() + northAt;
//  if (heading1 > 360) heading1 -= 360;
//
//  byte Dir;
//  float turnangle = 0;
//
//  while (abs(assigned_heading - heading1) > 3) {
//    if (heading1 < 180 && heading1 >= 0)
//    {
//      if (assigned_heading - heading1 > 0 && assigned_heading - heading1 < 180)
//      {
//        Dir = 0;//左转
//      } else
//      {
//        Dir = 1;//右转
//      }
//    }
//    else
//    {
//      if (assigned_heading < 360 && assigned_heading > 180)
//      {
//        if (assigned_heading - heading1 > 0)
//          Dir = 1;
//        else
//          Dir = 0;//左转
//      }
//      else if (assigned_heading <= 180 && assigned_heading > 0)
//      {
//        if (heading1 - assigned_heading < 180)
//        {
//          Dir = 0;//左转
//        }
//        else
//          Dir = 1;
//      }
//    }
//    if(abs(assigned_heading - heading1)>180)
//    {
//      turnangle = 360 - abs(assigned_heading - heading1);
//    }
//    else
//    {
//      turnangle = abs(assigned_heading - heading1);
//    }
//    Serial.print("AH:");
//    Serial.print(assigned_heading);
//    Serial.print(" H:");
//    Serial.print(heading1);
//    Serial.print("  Dir:");
//    Serial.println(Dir);
//    //    digitalWrite(4, Dir);
//    //    digitalWrite(7, !Dir);
//    //analogWrite(MOTORL_PIN, constrain(MIN_TURN_W+3*abs(assigned_heading - heading1), 0, 255));
//    //analogWrite(MOTORR_PIN, constrain(MIN_TURN_W+3*abs(assigned_heading - heading1), 0, 255));
//    digitalWrite(4, 1);
//    digitalWrite(7, 1);
//    if (Dir)
//    {
//      analogWrite(MOTORL_PIN, constrain(MIN_TURN_W + 2 * turnangle, 0, 255));
//      analogWrite(MOTORR_PIN, 0);
//    }
//    else
//    {
//      analogWrite(MOTORR_PIN, constrain(MIN_TURN_W + 2 * turnangle, 0, 255));
//      analogWrite(MOTORL_PIN, 0);
//    }
//
//
//    compass.read();
//    heading1 = compass.heading() + northAt;
//    if (heading1 > 360) heading1 -= 360;
//
//  }
//  Serial.print("turn around success! now heading:");
//  Serial.println(heading1);
//}

void turnAround1() {
  Serial.println("start turning around");
  double hea1, hea2;
  double assigned_heading, heading1;
  if (lastPoint_X == -1) {
    hea1 = 0; hea2 = 0;
    assigned_heading = atan2(nextPoint_X - currentPoint_X, nextPoint_Y - currentPoint_Y) * 180 / PI;
    if (assigned_heading < 0) assigned_heading += 360; // assigned_heading: [0, 360)

    compass.read();
    heading1 = compass.heading() + northAt;
    if (heading1 > 360) heading1 -= 360;
  }
  else {
    hea1 = atan2(currentPoint_X - lastPoint_X, currentPoint_Y - lastPoint_Y) * 180 / PI;
    if (hea1 < 0) hea1 += 360; // assigned_heading: [0, 360)
    hea2 = atan2(nextPoint_X - currentPoint_X, nextPoint_Y - currentPoint_Y) * 180 / PI;
    if (hea2 < 0) hea2 += 360; // assigned_heading: [0, 360)


    compass.read();
    heading1 = compass.heading() + northAt;
    if (heading1 > 360) heading1 -= 360;
    assigned_heading = heading1 + hea2 - hea1;
    if (assigned_heading > 360) assigned_heading -= 360;
    else if (assigned_heading < 0) assigned_heading += 360;

  }


  byte Dir;
  //
  Serial.print("hea1:");
  Serial.print(hea1);
  Serial.print("hea2:");
  Serial.print(hea2);
  Serial.print("AH:");
  Serial.print(assigned_heading);
  Serial.print(" H:");
  Serial.print(heading1);

  while (abs(assigned_heading - heading1) > 3) {
    if (assigned_heading - heading1 > 0)
    {
      Dir = 1;
    } else {
      Dir = 0;
    }
    Serial.print("AH:");
    Serial.print(assigned_heading);
    Serial.print(" H:");
    Serial.print(heading1);
    Serial.print("  Dir:");
    Serial.println(Dir);
    digitalWrite(4, 1);
    digitalWrite(7, 1);
    if (Dir)
    {
      analogWrite(MOTORL_PIN, constrain(MIN_TURN_W + 2 * abs(assigned_heading - heading1), 0, 255));
      analogWrite(MOTORR_PIN, 0);
    }
    else
    {
      analogWrite(MOTORR_PIN, constrain(MIN_TURN_W + 2 * abs(assigned_heading - heading1), 0, 255));
      analogWrite(MOTORL_PIN, 0);
    }

    compass.read();
    heading1 = compass.heading() + northAt;
    if (heading1 > 360) heading1 -= 360;

  }
  Serial.print("turn around success! now heading:");
  Serial.println(heading1);
}

//void turnAround2() {
//  Serial.println("start turning around");
//  double hea1, hea2;
//  double assigned_heading, heading1;
//  if (lastPoint_X == -1) {
//    hea1 = 0; hea2 = 0;
//    assigned_heading = atan2(nextPoint_X - currentPoint_X, nextPoint_Y - currentPoint_Y) * 180 / PI;
//    if (assigned_heading < 0) assigned_heading += 360; // assigned_heading: [0, 360)
//
//  }
//  else {
//    hea1 = atan2(currentPoint_X - lastPoint_X, currentPoint_Y - lastPoint_Y) * 180 / PI;
//    if (hea1 < 0) hea1 += 360; // assigned_heading: [0, 360)
//    hea2 = atan2(nextPoint_X - currentPoint_X, nextPoint_Y - currentPoint_Y) * 180 / PI;
//    if (hea2 < 0) hea2 += 360; // assigned_heading: [0, 360)
//
//    compass.read();
//    heading1 = compass.heading() + northAt;
//    if (heading1 > 360) heading1 -= 360;
//
//    assigned_heading = heading1 + hea2 - hea1;
//    if (assigned_heading > 360) assigned_heading -= 360;
//    else if (assigned_heading < 0) assigned_heading += 360;
//
//  }
//
//  byte Dir;
//  float turnangle = 0;
//  //
//  // Serial.print("hea1:");
//  //    Serial.print(hea1);
//  //    Serial.print("hea2:");
//  //    Serial.print(hea2);
//  //    Serial.print("AH:");
//  //    Serial.print(assigned_heading);
//  //    Serial.print(" H:");
//  //    Serial.print(heading1);
//
//  while (abs(assigned_heading - heading1) > 3) {
//    if (heading1 < 180 && heading1 >= 0)
//    {
//      if (assigned_heading - heading1 > 0 && assigned_heading - heading1 < 180)
//      {
//        Dir = 0;//左转
//      } else
//      {
//        Dir = 1;//右转
//      }
//    }
//    else
//    {
//      if (assigned_heading < 360 && assigned_heading > 180)
//      {
//        if (assigned_heading - heading1 > 0)
//          Dir = 1;
//        else
//          Dir = 0;//左转
//      }
//      else if (assigned_heading <= 180 && assigned_heading > 0)
//      {
//        if (heading1 - assigned_heading < 180)
//        {
//          Dir = 0;//左转
//        }
//        else
//          Dir = 1;
//      }
//    }
//    if(abs(assigned_heading - heading1)>180)
//    {
//      turnangle = 360 - abs(assigned_heading - heading1);
//    }
//    else
//    {
//      turnangle = abs(assigned_heading - heading1);
//    }
//    Serial.print("AH:");
//    Serial.print(assigned_heading);
//    Serial.print(" H:");
//    Serial.print(heading1);
//    Serial.print("  Dir:");
//    Serial.println(Dir);
//    digitalWrite(4, 1);
//    digitalWrite(7, 1);
//    if (Dir)
//    {
//      analogWrite(MOTORL_PIN, constrain(MIN_TURN_W + 2 * turnangle, 0, 255));
//      analogWrite(MOTORR_PIN, 0);
//    }
//    else
//    {
//      analogWrite(MOTORR_PIN, constrain(MIN_TURN_W + turnangle, 0, 255));
//      analogWrite(MOTORL_PIN, 0);
//    }
//
//    compass.read();
//    heading1 = compass.heading() + northAt;
//    if (heading1 > 360) heading1 -= 360;
//
//  }
//  Serial.print("turn around success! now heading:");
//  Serial.println(heading1);
//}

void goStraight(double distance) {
  Serial.println("start go straight");
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
    sDis = (rightWheel + leftWheel) / 2 / 9.11;
    int wheelErro = -leftWheel + rightWheel;
    e[2] = e[1];
    e[1] = e[0];
    e[0] = wheelErro; //Input;
    PID_Output = KP * (e[0] - e[1]) + KI * (e[0]) - KD * (e[0] - 2 * e[1] + e[2]) + PID_Output;

    analogWrite(MOTORL_PIN, constrain(255 + int(PID_Output), 0, 255));
    analogWrite(MOTORR_PIN, constrain(255 - int(PID_Output), 0, 255));

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
  analogWrite(MOTORL_PIN, 0);
  analogWrite(MOTORR_PIN, 0);
  delay(i * 1000);
}

void count1() {
  rightWheel++;
}
void count2() {
  leftWheel++;
}
