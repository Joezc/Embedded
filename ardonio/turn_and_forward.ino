#include <Wire.h>
#include <LSM303.h>
#include<NewPing.h>


#define KP 3
#define KI 0
#define KD 0
#define motor1 5
#define motor2 6

#define dead_zone 2 //compass's dead zone

LSM303 compass;
int assigned_heading = 0;


#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(41, 42, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(43, 44, MAX_DISTANCE),
  NewPing(45, 46, MAX_DISTANCE),
};

  //Define Variables we'll be connecting to
  float Input;
  float Output = 0;
  float error;

  int c = 0;
  int speedlevel = 0;
  int basicspeed;
#define Minoutput  140

  float e[3] = {0, 0, 0};
  float smooth[5] = {0, 0, 0, 0, 0};

  byte isforward = 0;
  byte enable = 0;
  byte is_init;
  byte Direction = 0;


  const byte ledPin = 13;
  const byte interruptPin = 2;
  const byte interruptPin2 = 3;

  unsigned long i = 0;
  unsigned long j = 0;
  double distance ;
  double distance2;
  double temp;
  double temp2;

  unsigned long LoopTime, LastLoop, LoopCount;

  void setup() {

    pinMode(interruptPin, INPUT_PULLUP);
    pinMode(interruptPin2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), blink1, RISING);
    attachInterrupt(digitalPinToInterrupt(interruptPin2), blink2, RISING);

    for (int i = 4; i <= 7; i++) //Pin 4 to 7 are used
      pinMode(i, OUTPUT);

    pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
    for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
      pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

    is_init = 0;
    Serial.begin(9600);
    Wire.begin();
    compass.init();
    compass.enableDefault();

    /*
      Calibration values; the default values of +/-32767 for each axis
      lead to an assumed magnetometer bias of 0. Use the Calibrate example
      program to determine appropriate values for your particular unit.
    */
    compass.m_min = (LSM303::vector<int16_t>) {
      -32767, -32767, -32767
    };
    compass.m_max = (LSM303::vector<int16_t>) {
      +32767, +32767, +32767
    };

    is_init = 0;
    basicspeed = 0;
    speedlevel = 0;

    if (is_init == 0)
    {
      assigned_heading = 0;
      enable = 1;
      isforward = 0;
      Serial.println("initializing");
    }

  }

  void loop() {
    compass.read();

    /*
      When given no arguments, the heading() function returns the angular
      difference in the horizontal plane between a default vector and
      north, in degrees.

      The default vector is chosen by the library to point along the
      surface of the PCB, in the direction of the top of the text on the
      silkscreen. This is the +X axis on the Pololu LSM303D carrier and
      the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
      carriers.

      To use a different vector as a reference, use the version of heading()
      that takes a vector argument; for example, use

      compass.heading((LSM303::vector<int>){0, 0, 1});

      to use the +Z axis as a reference.
    */



    float heading = compass.heading();



    Input = (assigned_heading - heading);
    if (Input > 180)
    {
      Input = Input - 360;
    }
    else if (Input < -180)
    {
      Input = Input + 360;
    }

    byte init_temp = is_init;

    if (abs(Input) < dead_zone)
    {
      //   byte init_temp=is_init;
      is_init = 1;
      if (is_init == 1)
 //       Serial.println("initlized");
      isforward = 1;
      //    Serial.println("isfowrad = 1 .NO1");
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
            assigned_heading = assigned_heading + 90;
            speedlevel=0;
            basicspeed=0;
            Serial.println("reveived");
            Serial.println(assigned_heading);
            break;
          case 'd':
            enable = 1;
            isforward = 0;
            assigned_heading = assigned_heading - 90;
            speedlevel=0;
            basicspeed=0;
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
              if(speedlevel<0)w
              assigned_heading = assigned_headingw - 180;
            basicspeed = abs(speedlevel) * 51;
            if (speedlevel < 0)
              Direction = 0;
            break;
          case 'x':
            speedlevel = 0;
            analogWrite(5, 0); //0 power == stop
            analogWrite(6, 0);
            enable = 0;
            break;
        }
      }
    }
    /*

      if(is_init)
      {
      if(0<obst&&obst<10)
      {
       assigned_heading+=90;

       if(assigned_heading>=360)
       assigned_heading-=360;
      }
      else if(Input<8)
      isforward=1;
      }
    */

    if (abs(Input) < dead_zone)
      isforward = 1;


    //Serial.println(Input);
    e[2] = e[1];
    e[1] = e[0];
    e[0] = Input;

    //Serial.println(e[0]);
    Output = KP * (e[0] - e[1]) + KI * (e[0]) + KD * (e[0] - 2 * e[1] + e[2]) + Output;

    // Output=constrain(Output,-50,50);
    //  Serial.println(Output);
    temp = distance;
    distance = i * 22 / 48 / 4;
    temp2 = distance2;
    distance2 = j * 22 / 48 / 4;

    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
      if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
    //    if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                          // Sensor being accessed.
        cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }

    LoopTime = millis() - LastLoop;
    LoopCount = LoopCount + LoopTime;

    if (LoopCount > 200) {
      Serial.print("H:");
      Serial.print(heading);
      Serial.print("  LD: ");
      Serial.print(distance);
      //Serial.print("\t");
      Serial.print("cm  RD:");
      Serial.print(distance2);
      Serial.print("cm  SL:");
      Serial.print(cm[0]);
      Serial.print("cm");
      Serial.print("cm  SR:");
      Serial.print(cm[1]);
      Serial.print("cm");
      Serial.print("cm  SF:");
      Serial.print(cm[2]);
      Serial.println("cm");
      LoopCount = 0;
    }
    motoroutput(enable, isforward, int(Output));
    //delay(100);


    LastLoop = millis();

  }
  void motoroutput(byte ena, byte isforward, int turningspeed)
  {
    if (ena == 1)
    {

      if (isforward == 0)
      {
     //   Serial.println("turning");
        byte Dir = LOW;
        if (turningspeed > 0)
        {
          Dir = HIGH;
        }

        if (abs(turningspeed) > 5) {
          digitalWrite(4, !Dir);
          digitalWrite(7, Dir);
          //    turningspeed=constrain(Minoutput+5*abs(turningspeed),0,255);
          analogWrite(motor1, 200);//constrain(Minoutput + 2 * abs(turningspeed), 0, 255));
          analogWrite(motor2, 200);//constrain(Minoutput + 2 * abs(turningspeed), 0, 255));
        }
      }
      if (isforward == 1)
      {
        //   Serial.print("_going forward_");
        //    Serial.println(basicspeed);
        if (Direction)
        {
          digitalWrite(4, 0);
          digitalWrite(7, 0);
        }
        else
        {
          digitalWrite(4, 1);
          digitalWrite(7, 1);
        }
        analogWrite(motor1, constrain(basicspeed + turningspeed/2, 0, 255));
        analogWrite(motor2, constrain(basicspeed - turningspeed/2, 0, 255));
        //  Serial.println(basicspeed+turningspeed);
        //  Serial.println(basicspeed-turningspeed);
      }

    }
  }

  void blink1() {
    i++;

    //state = !state;
    //digitalWrite(ledPin, state);
  }
  void blink2() {
    j++;
  }

  void echoCheck() { // If ping received, set the sensor distance to array.
    if (sonar[currentSensor].check_timer())
      cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }

