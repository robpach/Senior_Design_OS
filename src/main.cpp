// Created by Robert Pach
// February 16, 2026
/*
This program receives serial input form the control touchscreen device and
interprets the commands to control the two encoder motors connected
to the RR-RP SCARA.
There is option for a homing sequence, demo sequence, and manual control
*/

// LIBRARIES //
#include <Arduino.h>
#include <QuickPID.h>
#include <Wire.h>
#include <Adafruit_PCF8574.h> // might be 5 instead fo 4 at the end


// Function prototypes
void MotorDirection(int motor, int direction);
void HomeMotors();
void zAxis(int toggleA);
void suction(int toggleS);
void updatePosition(int x, int y);
void inverseCalc(float x, float y, int pointNum);
bool PositionChange1(int target);
bool PositionChange2(int target);
int radToPos(float radians);
void countChangeA1();
void countChangeB1();
void countChangeA2();
void countChangeB2();
void receiveEvent(int howMany);
void linspace(float start, float end, int numPoints, float *array);
void CommTask(void *pvParameters);
void MoveTo(float endX, float endY);
void ForwardCalc(float theta1, float THETA2);
void syncCurrentPosition();
void MoveToPTP(float endX, float endY);
void IncrementAngles(int rx_x, int rx_y);

// Intitialize the GPIO expansion
Adafruit_PCF8574 pcf;

// Mutex and Multithreading
SemaphoreHandle_t dataMutex;
struct ControlData
{
  int x, y, a, s, h, d, k, ma, mb;
} latestData;
void CommTask(void *pvParameters);
#define TICK_5MS pdMS_TO_TICKS(5)

// VARIABLE DEFINITIONS //

const float deg2rad = PI / 180;
const float rad2deg = 180 / PI;

// Forward Kinematic temp variables
float Xcalc, Ycalc;

// Kinematic arrays
const int maxPoints = 1000;
float pointDensity = 400.0; 
float dist = 0.0;
int pointCount = 0;
float xPoints[maxPoints];
float yPoints[maxPoints];
double theta1[maxPoints];
double THETA2[maxPoints];

// Demo position parameters
float carriageX = 0.0;
float carriageY = 0.4; // both in meters, these are just test values
int receivedPoints = 0;
float pictureAngle = PI;
int pictureCount = radToPos(pictureAngle);

// Position
volatile float currentPosX = 0.0;
volatile float currentPosY = 0.5;
int last_rx_x = 0;
int last_rx_y = 0;
int last_rx_a = 0;
int last_rx_s = 0;
int last_rx_ma = 0;
int last_rx_mb = 0;

// motor position for homing and control
long theta1_home, theta2_home;
long theta1_target, theta2_target;


// link lengths all in m
// the commented values are the link lengths of nicky's model

// values given by Nicky on 4/22/2026
const float r1 = 0.0360;
const float r2 = 0.1230;
const float r3 = 0.1260;  
const float r4 = 0.1319; 
int theta2_0 = 0;
float L1 = 0.188; // 0.186 before
float L2 = 0.1624; 
float theta2_max = 160 * deg2rad;


/*
// These values were manually measured
const float r1 = 0.0355;
const float r2 = 0.1223;
const float r3 = 0.1256;  
const float r4 = 0.1305; 
int theta2_0 = 0;
float L1 = 0.186;    
float L2 = 0.1625; 
float theta2_max = 160 * deg2rad;
*/

/////////////////////////////////
/* PINOUT as of 3/9/2026
each pcf pin is minus 1 (e.g. P1 = 0)
Homing switches (normally LOW): 
sw1 - RX (38)
sw2 - TX (39)
 
On-board high current switch:
vacuum control - P6 
 
Motor Driver (VNH5019ATR-E) A:
INA - P5 
ENA - P1
PWM - A4 (14)
CS -A5 (8)
ENB - SCK (36)
INB - MO (35)
 
Motor Driver (VNH5019ATR-E) B:
INA -  P4
ENA - P2   
PWM - MI (37)
CS - D5 
ENB - D6
INB - D9
 
Encoder A:
white - A0 (18) B
yellow - A1 (17) A
 
Encoder B:
white - A2 (16) B
yellow - A3 (15) A
*/
/*pcf list
motor1dirA
motor1enableA
motor2dirA
motor2enableA
vacuum
*/

// motor1 - the motor that controls theta1
int motor1PWM = 14;
int motor1dirA = 4; //PCF // HIGH always increases position1, if black top and red bottom then moves CCW
int motor1dirB = 35;
int motor1encA = 18; // solder these onto esp32
int motor1encB = 17; 
int motor1enableA = 0; // PCF
int motor1enableB = 6;
int motor1CS = 8;

// motor2 - the motor that controls theta2
int motor2PWM = 37;
int motor2dirA = 3; // PCF // HIGH always increases position2, if red top and black bottom then moves CCW
int motor2dirB = 9;
int motor2encA = 15; // solder these onto esp32
int motor2encB = 16;
int motor2enableA = 1; // PCF
int motor2enableB = 6;
int motor2CS = 5;

// Limit switches
int limit1 = 38; // this limit is for theta1
int limit2 = 39; // this limit is for theta2

// solenoid valves
int cylinder = 7; // pcf
int vacuum_sol = 6; // pcf

// arrival flags
bool arrive1 = false;
bool arrive2 = false;

// vacuum
int vacuum = 5; // PCF


// camera variable
float camX[100];
float camY[100];
float receivedX, receivedY;
volatile bool allPointsReceived = false;

// encoder parameters
volatile long position1 = 0;  // position for theta1
volatile long position2 = 0;  // position for theta2
const int totalCounts = 9600; // in encoder counts

// Limits in encoder counts for each motor
// motor 1
const float theta1_min = 0.0;
const int minPos1 = -(int)((float)totalCounts * (theta1_min / 360.0));
const int maxPos1 = (180.0 / 360.0) * (float)totalCounts;
// motor 2
const float theta2_min = 9.0;
const int minPos2 = (theta2_min / 360.0) * (float)totalCounts;
const int maxPos2 = ((180.0 / 360.0) * (float)totalCounts) + maxPos1;


// Home Positions - CHANGE THESE TO ACTUAL X AND Y AFTER HOMING
float homePosX = 0.0; // add the link lengths together
float homePosY = 0.0; // 0

// PID PARAMETERS //

// MoveTo parameters
float Kp1_MT = 0.05, Ki1_MT = 0.0, Kd1_MT = 0.001;
float Kp2_MT = 0.08, Ki2_MT = 0.0, Kd2_MT = 0.001;

// MoveToPTP parameters
float Kp1_PTP = 0.05, Ki1_PTP = 0.0, Kd1_PTP = 0.001;
float Kp2_PTP = 0.05, Ki2_PTP = 0.0, Kd2_PTP = 0.001;

// Motor control parameters 
float Kp1 = 0.10, Ki1 = 0.0, Kd1 = 0.001; // Kp = 0.05 Kd = 0.001
float Kp2 = 0.10, Ki2 = 0.0, Kd2 = 0.001;

// XY control parameters
float Kp1_inv = 0.10, Ki1_inv = 0.0, Kd1_inv = 0.0001; // Kp = 0.10 Kd = 0.0001
float Kp2_inv = 0.10, Ki2_inv = 0.0, Kd2_inv = 0.0001;

// motor 1
float Input1, Output1, Setpoint1;
QuickPID PID1(&Input1, &Output1, &Setpoint1);

// motor 2
float Input2, Output2, Setpoint2;
QuickPID PID2(&Input2, &Output2, &Setpoint2);


// speed scaling factor for PID output, between 0 and 1
// this just limits the maximum speed
float speedScale = 0.2;
// I can also change pointDensity to affect smoothness/speed

// Serial
int rx_x, rx_y, rx_a, rx_s, rx_h, rx_d, rx_k, rx_ma, rx_mb;

enum MachineStates
{
  Waiting,
  Home,
  Demo,
  InverseControl,
  Kinematics,
  MotorControl
};

MachineStates currentState = Waiting;


void setup()
{

  // SERIAL INITIALIZATION //
  Serial.begin(115200);
  // while (!Serial);
  delay(1000);
  Serial2.begin(115200, SERIAL_8N1, 10, 12); // rx, tx
  Serial2.println("System Online. Listening for other ESP32 on Pins 10/12...");

  // Built in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // I2C initialization
  Wire.begin(); // I2C_ADDR is slave address
  //Wire.onReceive(receiveEvent);

  // PCF initialization
  while (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    delay(100);
  }
  Serial.println("PCF connected");
  // PIN INITIALIZATION //

  /*pcf list
  motor1dirA X
  motor1enableA X
  motor2dirA X
  motor2enableA X
  vacuum
  */
 
  // motor 1
  pinMode(motor1PWM, OUTPUT);
  pcf.pinMode(motor1dirA, OUTPUT);
  pinMode(motor1dirB, OUTPUT);
  pinMode(motor1encA, INPUT_PULLDOWN);
  pinMode(motor1encB, INPUT_PULLDOWN);

  pcf.pinMode(motor1enableA, OUTPUT);
  pinMode(motor1enableB, OUTPUT);
  pcf.digitalWrite(motor1enableA, HIGH);
  digitalWrite(motor1enableB, HIGH);
  pcf.digitalWrite(motor1dirA, HIGH);
  digitalWrite(motor1dirB, LOW);

  // motor 2
  pinMode(motor2PWM, OUTPUT);
  pcf.pinMode(motor2dirA, OUTPUT);
  pinMode(motor2dirB, OUTPUT);
  pinMode(motor2encA, INPUT_PULLDOWN);
  pinMode(motor2encB, INPUT_PULLDOWN);

  pcf.pinMode(motor2enableA, OUTPUT);
  pinMode(motor2enableB, OUTPUT);
  pcf.digitalWrite(motor2enableA, HIGH);
  digitalWrite(motor2enableB, HIGH);
  pcf.digitalWrite(motor2dirA, HIGH);
  digitalWrite(motor2dirB, LOW);

  // limit switches
  pinMode(limit1, INPUT_PULLDOWN);
  pinMode(limit2, INPUT_PULLDOWN);

  // suction and actuation
  pcf.digitalWrite(cylinder, LOW);
  pcf.pinMode(vacuum, OUTPUT);
  pcf.pinMode(vacuum_sol, OUTPUT);
  pcf.pinMode(cylinder, OUTPUT);
  

  // i2c lines
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);

  // current sense pins
  pinMode(motor1CS, INPUT_PULLDOWN);
  pinMode(motor2CS, INPUT_PULLDOWN);

  // ATTACHING INTERRUPTS //
  attachInterrupt(digitalPinToInterrupt(motor1encA), countChangeA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor1encB), countChangeB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2encA), countChangeA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2encB), countChangeB2, CHANGE);

  // MULTITHREADING //
  dataMutex = xSemaphoreCreateMutex();

  // Create the Comm Task on Core 0
  xTaskCreatePinnedToCore(
      CommTask,   /* Function to implement the task */
      "CommTask", /* Name of the task */
      10000,      /* Stack size in words */
      NULL,       /* Task input parameter */
      1,          /* Priority of the task */
      NULL,       /* Task handle */
      0);         /* Core ID (0) */

  // PID INITIALIZATION //
  // motor1
  Input1 = position1;
  PID1.SetTunings(Kp1, Ki1, Kd1);
  PID1.SetMode(PID1.Control::automatic);
  PID1.SetAntiWindupMode(PID1.iAwMode::iAwCondition);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTimeUs(2000);
  // motor2
  Input2 = position2;
  PID2.SetTunings(Kp2, Ki2, Kd2);
  PID2.SetMode(PID2.Control::automatic);
  PID2.SetAntiWindupMode(PID2.iAwMode::iAwCondition);
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTimeUs(2000);

  // To test the demo, uncomment this line
  allPointsReceived = true;
}

void loop()
{
  
  receivedPoints = 2;
  camX[0] = 0.22;
  camY[0] = 0.30;
  camX[1] = -0.22;
  camY[1] = 0.30;

  switch (currentState)
  {
  case Waiting:

    if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE)
    {
      rx_x = latestData.x;
      rx_y = latestData.y;
      rx_a = latestData.a;
      rx_s = latestData.s;
      rx_h = latestData.h;
      rx_d = latestData.d;
      rx_k = latestData.k;
      rx_ma = latestData.ma;
      rx_mb = latestData.mb;
      xSemaphoreGive(dataMutex);

      // Logic to switch states
      if (rx_h == 1)
      {
        latestData.h = 0;
        currentState = Home;
      }
      else if (rx_d == 1)
      {
        currentState = Demo;
      }
      else if (rx_k == 1)
      {
        currentState = Kinematics;
      }
      else if (last_rx_x != rx_x || last_rx_y != rx_y)
      {
        last_rx_x = rx_x;
        last_rx_y = rx_y;
        last_rx_a = rx_a;
        last_rx_s = rx_s;
        currentState = InverseControl;
      }
      else if (last_rx_ma != rx_ma || last_rx_mb != rx_mb)
      {
        last_rx_ma = rx_ma;
        last_rx_mb = rx_mb;
        currentState = MotorControl;
      }
      else if (last_rx_a != rx_a || last_rx_s != rx_s) 
      {
        last_rx_a = rx_a;
        last_rx_s = rx_s;
        zAxis(rx_a);
        suction(rx_s);
      }
    }

    break;

  case Home:
    HomeMotors();
    currentState = Waiting;
    break;

  case Demo:

    // We need to tell the robot to move out of the way of the mechanism so that the camera can take a good pic
    // THEN, the esp32 should send a signal through Serial2 giving the camera a green light  
    // At this point, the mechanism should be homed

    Serial2.println("picture");

    PID1.SetTunings(Kp1_PTP, Ki1_PTP, Kd1_PTP);
    PID2.SetTunings(Kp2_PTP, Ki2_PTP, Kd2_PTP);
    // create some sort of statement that guarantees all data is sent, aka camX[receivedPoints] && camY[receivedPoints] = 0
    if (!allPointsReceived)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (allPointsReceived)
    {
      const TickType_t actuationDelay = pdMS_TO_TICKS(300);
      for (int i = 0; i < receivedPoints; i++)
      {
        // move from current position to chip position and pick up
        MoveToPTP(camX[i], camY[i]);
        vTaskDelay(actuationDelay);
        suction(1);
        vTaskDelay(actuationDelay);
        zAxis(1);
        vTaskDelay(actuationDelay);
        zAxis(0);

        // delete this after testing
        vTaskDelay(pdMS_TO_TICKS(1000));

        // move to carriage and drop off
        MoveToPTP(carriageX, carriageY);
        vTaskDelay(actuationDelay);
        zAxis(1);
        vTaskDelay(actuationDelay);
        suction(0);
        vTaskDelay(actuationDelay);
        zAxis(0);

        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      receivedPoints = 0;
      latestData.d = 0;
      currentState = Waiting; 
      allPointsReceived = false;
    }
    break;

  case Kinematics:

    // This will be a kinematic demo that goes through a couple of points
    // it doesn't use actuation or suction

    // Point to Point movement //
    // set the PID parameters for long distance point call
    PID1.SetTunings(Kp1_PTP, Ki1_PTP, Kd1_PTP);
    PID2.SetTunings(Kp2_PTP, Ki2_PTP, Kd2_PTP);
    MoveToPTP(0.22, 0.35);
    vTaskDelay(pdMS_TO_TICKS(1000));
    MoveToPTP(-0.22, 0.35);
    vTaskDelay(pdMS_TO_TICKS(1000));
    MoveToPTP(0.0, 0.2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    MoveToPTP(0.0, 0.4);
    vTaskDelay(pdMS_TO_TICKS(1000));
    MoveToPTP(0.22, 0.30);
    // Linear Movement //
    // set the PID parameters for linear movement

    vTaskDelay(pdMS_TO_TICKS(2000));
    
    PID1.SetTunings(Kp1_MT, Ki1_MT, Kd1_MT);
    PID2.SetTunings(Kp2_MT, Ki2_MT, Kd2_MT);
    MoveTo(-0.22, 0.30);
    vTaskDelay(pdMS_TO_TICKS(1000));
    MoveTo(0.22, 0.30);
    
    latestData.k = 0;
    currentState = Waiting;

    break;

  case InverseControl:
  // look at the data being received from serial
    if (xSemaphoreTake(dataMutex, (TickType_t)0) == pdTRUE)
    {
      rx_x = latestData.x;
      rx_y = latestData.y;
      rx_a = latestData.a;
      rx_s = latestData.s;
      xSemaphoreGive(dataMutex);
    }

    PID1.SetTunings(Kp1_inv, Ki1_inv, Kd1_inv);
    PID2.SetTunings(Kp2_inv, Ki2_inv, Kd2_inv);

    // setpoints for inverse kinematic control
    updatePosition(rx_x, rx_y);
    inverseCalc(currentPosX, currentPosY, 1);
    //PID1.SetTunings(Kp1_inv, Ki1_inv, Kd1_inv);
    //PID2.SetTunings(Kp2_inv, Ki2_inv, Kd2_inv);
    Setpoint1 = constrain(radToPos(theta1[1]), minPos1, maxPos1);
    Setpoint2 = constrain(radToPos(THETA2[1]), minPos2, maxPos2);

    while (!PositionChange1(Setpoint1) | !PositionChange2(Setpoint2))
    {
      PositionChange1(Setpoint1);
      PositionChange2(Setpoint2);
      zAxis(rx_a);
      suction(rx_s);
      vTaskDelay(pdMS_TO_TICKS(2));
    }

    zAxis(rx_a);
    suction(rx_s);
    
    arrive1 = PositionChange1(Setpoint1);
    arrive2 = PositionChange2(Setpoint2);
    if (arrive1 && arrive2) {
      currentState = Waiting;
    }

    /*
    // printing deugging every 200 ms
    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime > 200)
    { // Only print 10 times per second
      syncCurrentPosition();
      Serial2.print("X: ");
      Serial2.print(currentPosX, 3);
      Serial2.print(" | Y: ");
      Serial2.println(currentPosY, 3);
      lastPrintTime = millis();
    }
    */
      break;

    case MotorControl:
    // look at the data being received from serial
      if (xSemaphoreTake(dataMutex, (TickType_t)0) == pdTRUE)
      {
        rx_ma = latestData.ma;
        rx_mb = latestData.mb;
        rx_a = latestData.a;
        rx_s = latestData.s;
        xSemaphoreGive(dataMutex);
      }

      PID1.SetTunings(Kp1, Ki1, Kd1);
      PID2.SetTunings(Kp2, Ki2, Kd2);    
      
      // setpoints for motor control
      IncrementAngles(rx_ma, rx_mb);
      Setpoint1 = constrain(theta1_target, minPos1, maxPos1);
      Setpoint2 = constrain(theta2_target + Setpoint1, minPos2, maxPos2);

      while (!PositionChange1(Setpoint1) | !PositionChange2(Setpoint2))
      {
        PositionChange1(Setpoint1);
        PositionChange2(Setpoint2);
        zAxis(rx_a);
        suction(rx_s);
        vTaskDelay(pdMS_TO_TICKS(2));
      }

      zAxis(rx_a);
      suction(rx_s);
      
      arrive1 = PositionChange1(Setpoint1);
      arrive2 = PositionChange2(Setpoint2);
      if (arrive1 && arrive2) {
        currentState = Waiting;
      }

      /*
      // printing deugging every 200 ms
      static uint32_t lastPrintTime = 0;
      if (millis() - lastPrintTime > 200)
      { // Only print 10 times per second
        syncCurrentPosition();
        Serial2.print("X: ");
        Serial2.print(currentPosX, 3);
        Serial2.print(" | Y: ");
        Serial2.println(currentPosY, 3);
        lastPrintTime = millis();
      }
      */
      break;
  }
}

// FUNCTION DEFINITIONS //

void MotorDirection(int motor, int direction)
{
  if (motor == 1)
  {
    if (direction == HIGH)
    {
      pcf.digitalWrite(motor1dirA, HIGH);
      digitalWrite(motor1dirB, LOW);
    }
    else if (direction == LOW)
    {
      pcf.digitalWrite(motor1dirA, LOW);
      digitalWrite(motor1dirB, HIGH);
    } else if (direction == 3) 
    {
      pcf.digitalWrite(motor1dirA, HIGH);
      digitalWrite(motor1dirB, HIGH);
    }
  }
  else if (motor == 2)
  {
    if (direction == HIGH)
    {
      pcf.digitalWrite(motor2dirA, HIGH);
      digitalWrite(motor2dirB, LOW);
    }
    else if (direction == LOW)
    {
      pcf.digitalWrite(motor2dirA, LOW);
      digitalWrite(motor2dirB, HIGH);
    } 
    else if (direction == 3) 
    {
      pcf.digitalWrite(motor2dirA, HIGH);
      digitalWrite(motor2dirB, HIGH);
    }
  }
}

void HomeMotors()
{
  MotorDirection(1, LOW);  // CW
  MotorDirection(2, LOW);  // CW

  suction(LOW);
  zAxis(LOW);

  // fully extend theta2, not homed just yet
  bool limitHit2 = digitalRead(limit2);
  MotorDirection(2, LOW);
  analogWrite(motor2PWM, 80);
  while (!limitHit2) {
    limitHit2 = digitalRead(limit2);
  }
  analogWrite(motor2PWM, 0);

  // home theta1 while giving some signal to theta2 so it doesn't crash
  analogWrite(motor2PWM, 30);
  bool limitHit1 = digitalRead(limit1);
  MotorDirection(1, LOW);
  analogWrite(motor1PWM, 60);
  while (!limitHit1) {
    limitHit1 = digitalRead(limit1);
  }
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  MotorDirection(1,3); // brake
  theta1_home = (-(float)totalCounts * (theta1_min / 360.0)); // change this potentially, it was at 17.0 before
  position1 = (int)theta1_home; // 17.5 degrees below parallel with x axis
  
  // home theta 2
  limitHit2 = digitalRead(limit2);
  MotorDirection(2, LOW);
  analogWrite(motor2PWM, 50);
  while (!limitHit2) {
    limitHit2 = digitalRead(limit2);
  }
  analogWrite(motor2PWM, 0);
  MotorDirection(2,3); // brake
  theta2_home = ((float)totalCounts * (theta2_min / 360.0));
  position2 = (int)theta2_home; // 7 degrees from parallel with theta1

  float start_theta1 = ((float)theta1_home / (float)totalCounts) * 2 * PI;
  float start_theta2 = ((float)theta2_home / (float)totalCounts) * 2 * PI;

  // set home position. ForwardCalc redefine Xcalc and Ycalc globals
  ForwardCalc(start_theta1, start_theta2);
  homePosX = Xcalc;
  homePosY = Ycalc;
  currentPosX = homePosX;
  currentPosY = homePosY;

  // Set the known home positions in encoder counts
  //theta1_home = position1;
  //theta2_home = position2;

  /*
  vTaskDelay(pdMS_TO_TICKS(500));
  // call the motors to the position for ensured accuracy
  while (!PositionChange1(theta1_home) | !PositionChange2(theta2_home))
  {
    PositionChange1(theta1_home);
    PositionChange2(theta2_home);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  */
  
  // PID testing
  /*
  MoveToPTP(0.0, 0.4);

  vTaskDelay(pdMS_TO_TICKS(1000));

  MoveToPTP(-0.3, 0.3);
  */

  //while (true);


}

void syncCurrentPosition() {
  // 1. Get current joint angles in radians from the volatile encoders
  float current_th1 = ((float)position1 / (float)totalCounts) * 2.0 * PI;
  float current_th2 = ((float)position2 / (float)totalCounts) * 2.0 * PI;

  // 2. Run your Forward Kinematics (this updates the globals Xcalc and Ycalc)
  ForwardCalc(current_th1, current_th2);

  // 3. Update the tracking variables
  currentPosX = Xcalc;
  currentPosY = Ycalc;
}

void zAxis(int toggleA)
{

  if (toggleA == 1)
  {
    pcf.digitalWrite(cylinder, HIGH);
    // Serial.println("Actuator On");
  }
  else if (toggleA == 0)
  {
    pcf.digitalWrite(cylinder, LOW);
    // Serial.println("Actuator Off");
  }
}

void suction(int toggleS)
{

  if (toggleS == 1)
  {
    pcf.digitalWrite(vacuum, HIGH);
    pcf.digitalWrite(vacuum_sol, LOW);
    // Serial.println("Suction On");
    // add a statement to turn off the positive pressure solenoid
  }
  else if (toggleS == 0)
  {
    pcf.digitalWrite(vacuum, LOW);
    pcf.digitalWrite(vacuum_sol, HIGH);
    // Serial.println("Suction Off");
    // add a statement to turn on the positive pressure solenoid
  }
}

void updatePosition(int x, int y)
{
  // translate the positions rx_x and rx_y into actual positions in meters
  // we need a scale factor here, rx_x and rx_y are just integers
  currentPosX = homePosX + x * 0.003;
  currentPosY = homePosY + y * 0.003;
  
  // add a statement controlling the out of bounds error.
  // if sqrt(pow(currentPosX,2) + pow(currentPosY,2)) > (r1+r2+r3+r4) then out of bounds

}

bool PositionChange1(int target)
{
  Setpoint1 = target;
  Input1 = position1;
  PID1.Compute();

  // 1. ARRIVAL CHECK FIRST
  if (abs(position1 - target) <= 10) // Window widened for arm weight
  {
    analogWrite(motor1PWM, 0);
    MotorDirection(1, 3); // brake
    Output1 = 0;       
    return true;         
  }

  // 2. DEADZONE (Only if we didn't arrive)
  if (Output1 != 0 && abs(Output1) <= 35)
  {
    Output1 = (Output1 >= 0) ? 40 : -40;
  }

  // 3. CONSTRAIN & WRITE
  Output1 = constrain(Output1, -255 * speedScale, 255 * speedScale);
  MotorDirection(1, Output1 >= 0 ? HIGH : LOW);
  analogWrite(motor1PWM, (int)abs(Output1));
  
  return false;
}

bool PositionChange2(int target)
{
  Setpoint2 = target;
  Input2 = position2;
  PID2.Compute();

  // 1. ARRIVAL CHECK FIRST
  if (abs(position2 - target) <= 10) // Window widened for arm weight
  {
    analogWrite(motor2PWM, 0);
    MotorDirection(2, 3); // brake
    Output2 = 0;          
    return true;         
  }

  // 2. DEADZONE (Only if we didn't arrive)
  if (Output2 != 0 && abs(Output2) <= 25)
  {
    Output2 = (Output2 >= 0) ? 30 : -30;
  }

  // 3. CONSTRAIN & WRITE
  Output2 = constrain(Output2, -255 * speedScale, 255 * speedScale);
  MotorDirection(2, Output2 >= 0 ? HIGH : LOW);
  analogWrite(motor2PWM, (int)abs(Output2));
  
  return false;
}

void countChangeA1()
{
  if (digitalRead(motor1encB) != digitalRead(motor1encA))
    position1--;
  else
    position1++;
}

void countChangeB1()
{
  if (digitalRead(motor1encB) == digitalRead(motor1encA))
    position1--;
  else
    position1++;
}

void countChangeA2()
{
  if (digitalRead(motor2encB) != digitalRead(motor2encA))
    position2++;
  else
    position2--;
}

void countChangeB2()
{
  if (digitalRead(motor2encB) == digitalRead(motor2encA))
    position2++;
  else
    position2--;
}

int angleToPos(int ang)
{
  int pos = (float(ang) / 360.0) * 9600.0;
  return pos;
}

int radToPos(float rad)
{
  return (rad / (2.0 * PI)) * totalCounts;
}

void PrintStats1()
{
  // motor 1
  Serial.print("M1Setpoint:  ");
  Serial.print(Setpoint1);
  Serial.print("   ");
  Serial.print("M1Position:  ");
  Serial.println(position1);
}

void PrintStats2()
{
  // motor 2
  Serial.print("M2Setpoint:  ");
  Serial.print(Setpoint2);
  Serial.print("   ");
  Serial.print("M2Position:  ");
  Serial.println(position2);
}

void Stop()
{
  analogWrite(motor1PWM, LOW);
  analogWrite(motor2PWM, LOW);
  /*
  pcf.digitalWrite(vacuum, LOW);
  delay(200);
  digitalWrite(cylinder, LOW);
  */
  Serial.println("Stopped.");
  currentState = Waiting;
}

void inverseCalc(float Px, float Py, int i)
{
  float d;
  float A2, B2, C2;
  float K1, K2;
  float A3, B3, C3;
  float u2, u3;
  float R = sqrt(sq(Px) + sq(Py));

  // Slot Translational Distance w/ Solution Selection
  if (R <= (L1 + r1) + L2)
  {
    d = 0;
  }
  else
  {
    d = -(L1 + r1) * cos(theta2_0) - L2 + sqrt(sq(R) - sq(sin(theta2_0)) * sq(L1 + r1));
  }

  // Solve for theta2 according to d
  float acos_calc = (sq(R) - sq(L1 + r1) - sq(L2 + d)) / (2 * (L2 + d) * (L1 + r1));
  float acos_value = constrain(acos_calc, -1.0, 1.0);
  float theta2 = acos(acos_value);

  if ((R > L1 + r2 + r3 + r4) || (abs(theta2) > theta2_max))
  {
    Serial2.println("Unachievable position, restart program");
    return;
  }

  // Solve for Theta1
  float beta = atan2(Py, Px);
  float gamma = atan2((L2 + d) * sin(theta2), L1 + r1 + ((L2 + d) * cos(theta2)));
  theta1[i] = beta - gamma;

  /*
  if (theta1[i] < 0) {
    theta1[i] += 2 * PI;
  }
  */

  // RRRR Inverse Kinematics; d = 0
  A2 = 2 * r2 * (-r1 - r4 * cos(theta2));
  B2 = -2 * r2 * r4 * sin(theta2);
  C2 = sq(r3) - sq(r2) - sq(r4) - sq(r1) - (2 * r1 * r4 * cos(theta2));
  // solve for u2
  float discriminant2 = sq(A2) + sq(B2) - sq(C2);
  if (discriminant2 < 0) discriminant2 = 0; 
  u2 = (B2 - sqrt(discriminant2)) / (C2 + A2 + 1e-6);

  // RRRP Inverse Kinematics; d =/= 0
  K1 = -(r4 + d) * cos(theta2_0) - r1;
  K2 = -(r4 + d) * sin(theta2_0);
  A3 = 2 * r2 * K1;
  B3 = 2 * r2 * K2;
  C3 = sq(r3) - sq(K1) - sq(K2) - sq(r2);
  // solve for u3
  u3 = (B3 - sqrt(sq(A3) + sq(B3) - sq(C3))) / (C3 + A3);

  // solve for THETA2
  if (d <= 1e-6)
  {
    THETA2[i] = 2.0 * atan(u2);
  }
  else
  {
    THETA2[i] = 2.0 * atan(u3);
  }

  // Differential drive compensation
  THETA2[i] += theta1[i];

  /*
  if (THETA2[i] < 0) {
    THETA2[i] += 2 * PI;
  }
  */
}

// input radians, use theta 1 and THETA2 from encoders
void ForwardCalc(float th1, float TH2) {
  // input of theta1 and THETA2 are in radians!
  // We will use constants r1, r2, r3, r4, theta2_0, L1, L2 as known

  // Intitialize all local variables
  float A_1, B_1, C_1, K, A, B, C, u, theta2, discriminant, d;

  // solve for variable length d
  A_1 = 1;
  B_1 = 2*(r1*cos(theta2_0) - r2*cos(TH2 - theta2_0) + r4);
  K = sq(r1) + sq(r2) - sq(r3) + sq(r4);
  C_1 = K - 2*(r1*r2*cos(TH2) - r1*r4*cos(theta2_0) + r2*r4*cos(TH2 - theta2_0));

  discriminant = sq(B_1) - 4*A_1*C_1;
  d = (-B_1 + sqrt(discriminant)) / (2*A_1);

  if (d < 0) {
    d = 0;
  }

  // solve for theta2 (lowercase)
  A = 2*r4*(r1-r2*cos(TH2));
  B = -2*r2*r4*sin(TH2);
  C = sq(r3) - sq(r2) - sq(r1) - sq(r4) + 2*r1*r2*cos(TH2);
  u = (B + sqrt(sq(A) + sq(B) - sq(C))) / (C + A);

  theta2 = 2*atan(u);

  if (d > 0) {
    theta2 = 0;
  }

  // calculate the points using theta1 and theta2
  Xcalc = (L1 + r1)*cos(th1) + (L2 + d)*cos(th1+theta2);
  Ycalc = (L1 + r1)*sin(th1) + (L2 + d)*sin(th1+theta2);

}

// This is the new and improved CommTask, to work with screen_4_27 waveshare project
void CommTask(void *pvParameters)
{

  String packetBuffer = "";

  for (;;)
  { // Infinite loop for the task
    while (Serial2.available() > 0)
    {
      char c = Serial2.read();

      if (c == '\n')
      {
        int tx, ty, ta, ts, th, td, tk, tma, tmb;
        float tx_f, ty_f;
        int matched_inverse = sscanf(packetBuffer.c_str(), "X%d Y%d A%d S%d H%d", &tx, &ty, &ta, &ts, &th);
        int matched_motor = sscanf(packetBuffer.c_str(), "MA%d MB%d A%d S%d H%d", &tma, &tmb, &ta, &ts, &th);
        int matched_demo = sscanf(packetBuffer.c_str(), "H%d D%d K%d", &th, &td, &tk);
        int matched_camera = sscanf(packetBuffer.c_str(), "X%f Y%f", &tx_f, &ty_f);


        if (matched_inverse == 5)
        {
          // Update the shared struct safely using a Mutex
          if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE)
          {
            latestData.x = tx;
            latestData.y = ty;
            latestData.a = ta;
            latestData.s = ts;
            latestData.h = th;
            // white on edge to receive from screen, green on edge to receive from esp32 
            xSemaphoreGive(dataMutex);
          }
        } else if (matched_motor == 5) {
          if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE)
          {
            latestData.ma = tma;
            latestData.mb = tmb;
            latestData.a = ta;
            latestData.s = ts;
            latestData.h = th;
            xSemaphoreGive(dataMutex);
          }
        } else if (matched_demo == 3) {
          if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE)
          {
            latestData.h = th;
            latestData.d = td;
            latestData.k = tk;
            xSemaphoreGive(dataMutex);
          }
        } else if (matched_camera == 2){
          // This is for the demo mode, where we just receive x and y coordinates until 0,0 is sent
          if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE)
          {
            latestData.x = tx_f;
            latestData.y = ty_f;
            // as each x and y point is sent, we need to add it to camX[i] an camY[i]
            if (receivedPoints < 100) // Assuming a maximum of 100 points
            {
              camX[receivedPoints] = tx_f;
              camY[receivedPoints] = ty_f;
              receivedPoints++;

              // print the received points
              Serial2.print("Received Point: ");
              Serial2.print(tx_f);
              Serial2.print(", ");
              Serial2.println(ty_f);
            }
            
            if (tx_f == 0.0 && ty_f == 0.0)
            {
              allPointsReceived = true;
            }
            xSemaphoreGive(dataMutex);
          }
        }
        packetBuffer = "";
      }
      else if (c != '\r')
      {
        packetBuffer += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Linear interpolation
void linspace(float start, float end, int numPoints, float *output)
{
  if (numPoints <= 0)
    return;

  if (numPoints == 1)
  {
    output[0] = start;
    return;
  }

  float step = (end - start) / (numPoints - 1);

  for (int i = 0; i < numPoints; i++)
  {
    output[i] = start + i * step;
  }
}

// Linear interpolates from start to end and moves the end effector
void MoveTo(float endX, float endY)
{
  syncCurrentPosition(); 
  PID1.SetTunings(Kp1_MT, Ki1_MT, Kd1_MT);
  PID2.SetTunings(Kp2_MT, Ki2_MT, Kd2_MT);
  dist = sqrt(pow((endX - currentPosX), 2) + pow((endY - currentPosY), 2));
  pointCount = int(dist * pointDensity);

  // edge case where distance is small
  if (pointCount < 2) {
    pointCount = 2;
  }

  if (pointCount > maxPoints) {
    pointCount = maxPoints;
  }

  linspace(currentPosX, endX, pointCount, xPoints);
  linspace(currentPosY, endY, pointCount, yPoints);

  for (int i = 0; i < pointCount; i++)
  {
    inverseCalc(xPoints[i], yPoints[i], i);
    Setpoint1 = constrain(radToPos(theta1[i]), minPos1, maxPos1);
    Setpoint2 = constrain(radToPos(THETA2[i]), minPos2, maxPos2);
    //use this loop instead of plain positionchange for more accuracy
    int timeout = 0; 
    while (!PositionChange1(Setpoint1) | !PositionChange2(Setpoint2))
    {
      PositionChange1(Setpoint1);
      PositionChange2(Setpoint2);
      vTaskDelay(pdMS_TO_TICKS(2));
      timeout++;
      if (timeout > 500) break;
    }
    
  }
  syncCurrentPosition();
}

void MoveToPTP(float endX, float endY) {
  syncCurrentPosition();
  PID1.SetTunings(Kp1_PTP, Ki1_PTP, Kd1_PTP);
  PID2.SetTunings(Kp2_PTP, Ki2_PTP, Kd2_PTP);
  inverseCalc(endX, endY, 0);
  Setpoint1 = constrain(radToPos(theta1[0]), minPos1, maxPos1);
  Setpoint2 = constrain(radToPos(THETA2[0]), minPos2, maxPos2);
  while (!PositionChange1(Setpoint1) | !PositionChange2(Setpoint2))
  {
    PositionChange1(Setpoint1);
    PositionChange2(Setpoint2);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  syncCurrentPosition();
}

void IncrementAngles(int rx_x, int rx_y) {
  theta1_target = theta1_home + (rx_x * 20);
  theta2_target = theta2_home + (rx_y * 20);
}