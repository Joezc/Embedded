#include <Wire.h>
#include <LSM303.h>
#include<NewPing.h>

/*****************************************************		Sonar	config	*********************************************************************/
#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(41, 42, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(43, 44, MAX_DISTANCE),
  NewPing(45, 46, MAX_DISTANCE)
};


#define LED_PIN     13
const byte interruptPin = 2;
const byte interruptPin2 = 3;
int turnAngle = 90; // angle when turn around

/**************************************************** Motor and motion config ****************************************************/
#define MOTORL_PIN 5
#define MOTORR_PIN 6

volatile byte isforward = 0;
byte enable = 1;
byte is_init = 0;
byte Direction = 0;

int speedlevel = 0;
int basicspeed = 0;

#define dead_zone 10 //compass's dead zone
#define MIN_TURN_W  180 // min output gonglv when turning around

float Output = 0;
float e[3] = {0, 0, 0};
#define KP 3
#define KI 0
#define KD 0

/************************************************** Mileage counter ***********************************************************/
float PULSE_PRE_METER = 48 * 8 / 22;
unsigned long leftWheel = 0;		// pules come from encodes
unsigned long rightWheel = 0;
unsigned long temp_L = 0;			//used to measure changes in pulse counters
unsigned long temp_R = 0;
double distance = 0, oldDistance = 0;
double axis_X = 0;
double axis_Y = 0;

/**************************************************  Compass config	************************************************88***********/
float dir_correction = 0;
LSM303 compass;
float assigned_heading = 0;
float heading = 0;
float current_h = 0;

/*************************************************	loop time control	***************************************************************/
unsigned long LoopTime, LastLoop, LoopCount;

/************************************************  Serial congfig *****************************************************************/
int now_x = 0;
int now_y = 0;
int now_heading = 0;
int sonar1 = 0;
int sonar2 = 0;
int sonar3 = 0;
int handshake = 0;//没握手
int readinit = 0;//未接受到初始化数据
//int handshake = 1; //假装握手
//int readinit = 1;//假装接受到初始化数据
byte initdata[6];
byte receive[2];
byte sendbytes[12] = {byte(now_x % 256), byte(now_x / 256), byte(now_y % 256), byte(now_y / 256), byte(now_heading % 256), byte(now_heading / 256), byte(sonar1 % 256), byte(sonar1 / 256), byte(sonar2 % 256), byte(sonar2 / 256), byte(sonar3 % 256), byte(sonar3 / 256)};

//坐标函数
void Coordination()
{
//  double zeng_liang = 0;
//  if (speedlevel >= 0)
//    zeng_liang = (rightWheel - temp_R + leftWheel - temp_L) / PULSE_PRE_METER;
//  else
//    zeng_liang = -(rightWheel - temp_R + leftWheel - temp_L) / PULSE_PRE_METER;
//
//  distance += abs(zeng_liang);
//  axis_X += (sin(current_h * 3.1416 / 180)) * zeng_liang;
//  axis_Y += (cos(current_h * 3.1416 / 180)) * zeng_liang;
//  temp_L = leftWheel;
//  temp_R = rightWheel;
  
  distance = (rightWheel + leftWheel)/2 /333 * 22.0;
  axis_X += (sin(current_h * 3.1416 / 180)) * (distance - oldDistance);
  axis_Y += (cos(current_h * 3.1416 / 180)) * (distance- oldDistance);
  oldDistance = distance;
//  rightWheel = 0;
//  leftWheel = 0;
}

void count1() {
  //if (isforward)
    rightWheel++;
}
void count2() {
  //if (isforward)
    leftWheel++;
}

void motoroutput(byte ena, byte isforward, int turningspeed)
{
  if (ena == 1)
  {
    if (isforward == 0)
    {
      //Serial.println("turning");
      byte Dir;
      if (turningspeed > 0)
      {
        Dir = HIGH;
      } else {
        Dir = LOW;
      }

      if (abs(turningspeed) > 12) {
        digitalWrite(4, Dir);
        digitalWrite(7, !Dir);
        analogWrite(MOTORL_PIN, constrain(MIN_TURN_W + abs(turningspeed), 0, 255));
        analogWrite(MOTORR_PIN, constrain(MIN_TURN_W + abs(turningspeed), 0, 255));
      } else {
        analogWrite(MOTORL_PIN, 0);
        analogWrite(MOTORR_PIN, 0);
      }
    }
    if (isforward == 1)
    {
      if (1)//Direction)
      {
        digitalWrite(4, 1);
        digitalWrite(7, 1);
        analogWrite(MOTORL_PIN, constrain(basicspeed + turningspeed, 0, 255));
        analogWrite(MOTORR_PIN, constrain(basicspeed - turningspeed, 0, 255));
      }
      else
      {
        digitalWrite(4, 0);
        digitalWrite(7, 0);
        analogWrite(MOTORL_PIN, constrain(basicspeed - turningspeed, 0, 255));
        analogWrite(MOTORR_PIN, constrain(basicspeed + turningspeed, 0, 255));
      }
    }
  }
}

void MotionControl()
{
  float Input = 0;
  Input = (assigned_heading - current_h);
  if (Input > 180)
  {
    Input = Input - 360;
  }
  else if (Input < -180)
  {
    Input = Input + 360;
  }

  if (abs(Input) < dead_zone)
  {
    is_init = 1;
    isforward = 1;
  }

  e[2] = e[1];
  e[1] = e[0];
  e[0] = Input;
  Output = KP * (e[0] - e[1]) + KI * (e[0]) - KD * (e[0] - 2 * e[1] + e[2]) + Output;

  motoroutput(enable, isforward, (int)Output);
}
void read_sensors()
{
  compass.read();
  heading = compass.heading();
  current_h = heading + dir_correction;
  if (current_h >= 360)
    current_h -= 360;
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
void setup()
{
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count1, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), count2, RISING);

  for (int i = 4; i <= 7; i++) //Pin 4 to 7 are used
    pinMode(i, OUTPUT);
  Serial.begin(9600);
  Serial3.begin(9600);
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
        handshake = 1;//握手完成
      }
    }
    else
    {
      Serial.println("Waiting for linking...");
    }
  }


  while (!readinit)
  {
    if (Serial3.available())
    {
      Serial.println("Connect is done!");
      Serial3.readBytes(initdata, 6);
      axis_X = initdata[0] * 256 + initdata[1];
      axis_Y = initdata[2] * 256 + initdata[3];
      dir_correction = initdata[4] * 256 + initdata[5];
      readinit = 1;//初始化数据接受完成
      Serial.print("initial x=");
      Serial.print(axis_X);
      Serial.print("initial y=");
      Serial.println(axis_Y);

      Serial3.print(255, HEX);
      Serial.println("Initial data has recieved");
    }
    else
    {
      Serial.println("Waiting for initial data");
    }
  }
  //Serial3.print("I got it!!");
  //Serial.println(inByte,HEX);

  Wire.begin();
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  isforward = 0;  //简易的初始化
  assigned_heading = 0;
  basicspeed = 0;
  speedlevel = 0;
}


void loop()
{
  read_sensors();
  Coordination();
  MotionControl();
  if (is_init) {
    if (handshake == 1 && readinit == 1)
    {
      //    Serial.println("Waiting for instruction...");
      if (Serial3.available())
      {
        Serial3.readBytes(receive, 2);
        assigned_heading = receive[0] * 256 + receive[1];
        Serial.print(receive[0],HEX);
        Serial.println(receive[1],HEX);
        speedlevel = 5;
        basicspeed = 240;
        enable = 1;
        isforward = 0;

        if (assigned_heading == 0xFFFF)
        {
          analogWrite(5, 0); //0 power == stop
          analogWrite(6, 0);
          speedlevel = 0;
          basicspeed = 0;
          enable = 0;
          isforward = 0;
         // Robot.motorsStop();
        }
      }
      sonar1 = (int)cm[0];
      sonar2 = (int)cm[1];
      sonar3 = (int)cm[2];
      now_x = (int)axis_X;
      now_y = (int)axis_Y;

      //      compass.read();
      //      now_heading = compass.heading();
      //      Serial.print("now_heading:");
      //      Serial.println(now_heading);
      //      Serial.print("now_x:");
      //      Serial.println(now_x);
      //      Serial.print("now_y:");
      //      Serial.println(now_y);
      //
      sendbytes[0] = byte(now_x / 256);
      sendbytes[1] = byte(now_x % 256);
      sendbytes[2] = byte(now_y / 256);
      sendbytes[3] = byte(now_y % 256);
      sendbytes[4] =  byte(now_heading / 256);
      sendbytes[5] =  byte(now_heading % 256);
      sendbytes[6] = byte(sonar1 / 256);
      sendbytes[7] = byte(sonar1 % 256);
      sendbytes[8] = byte(sonar2 / 256);
      sendbytes[9] = byte(sonar2 % 256);
      sendbytes[10] = byte(sonar3 / 256);
      sendbytes[11] = byte(sonar3 % 256);

      Serial3.write(sendbytes, 12);


    }
  }
  if (is_init)
  {
    if (Serial.available())
    {
      char input = Serial.read();
      switch (input) {
        case 'a':
          enable = 1;
          isforward = 0;
          assigned_heading = assigned_heading - turnAngle;
          if (assigned_heading < 0)
            assigned_heading += 360;
          Serial.println("reveived");
          Serial.println(assigned_heading);
          break;
        case 'd':
          enable = 1;
          isforward = 0;
          assigned_heading = assigned_heading + turnAngle;
          if (assigned_heading >= 360)
            assigned_heading -= 360;
          Serial.println("reveived");
          Serial.println(assigned_heading);
          break;
        case 'w':
          enable = 1;
          if (speedlevel < 5)
            speedlevel ++;
          if (speedlevel > 0)
            Direction = 1;
          basicspeed = abs(speedlevel * 51);
          break;
        case 's':
          enable = 1;
          if (speedlevel > - 5)
            speedlevel --;
          basicspeed = abs(speedlevel) * 51;
          if (speedlevel < 0)
            Direction = 0;
          break;
        case 'x':
          speedlevel = 0;
          basicspeed = abs(speedlevel) * 51;
          analogWrite(5, 0); //0 power == stop
          analogWrite(6, 0);
          enable = 0;
          break;
          //       default:

      }
    }
  }
  //*/

  LoopTime = millis() - LastLoop;
  LoopCount = LoopCount + LoopTime;
  if (LoopCount > 200) {
    compass.read();
    now_heading = compass.heading();
    Serial.print("now_heading:");
    Serial.print(now_heading);
    Serial.print(" assigned_heading: ");
    Serial.print(assigned_heading);
    //    double distance = i * 22 / 48 / 4;
    //   double distance2 = j * 22 / 48 / 4;
    Serial.print("  distance: ");
    Serial.print(distance);
    Serial.print("cm  X:");
    Serial.print(axis_X);
    Serial.print("cm  Y:");
    Serial.println(axis_Y);
    //    Serial.print(" init:");
    //    Serial.print(is_init);
    //    Serial.print(" ena:");
    //    Serial.print(enable);
    Serial.print(" cm  S2:");
    Serial.print(cm[2]);
    Serial.println("cm");

    Serial.print("Sendbytes:");
    for (int k = 0; k < 12; k++)
    {
      Serial.print(sendbytes[k]);
      Serial.print(' ');
    }

    Serial.println("senddata print done");

    LoopCount = 0;
  }
  LastLoop = millis();
}


