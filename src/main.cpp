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

// Intitialize the GPIO expansion
Adafruit_PCF8574 pcf;

// Mutex and Multithreading
SemaphoreHandle_t dataMutex;
struct ControlData
{
  int x, y, a, s, h, d;
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
float pointDensity = 50.0;
float dist = 0.0;
int pointCount = 0;
float xPoints[maxPoints];
float yPoints[maxPoints];
double theta1[maxPoints];
double THETA2[maxPoints];

// Demo position parameters
float carriageX = 0.0;
float carriageY = 0.3; // both in meters, these are just test values
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

// link lengths all in m
// the commented values are the link lengths of nicky's model
// CALIBRATE R1 ONCE ASSEMBLED
/*
// values given by Nicky, not necessarily wrong, but wanted to try my own
const float r1 = 0.04717; // 0.;
const float r2 = 0.16117; // 0.123;
const float r3 = 0.1651;  // 0.126;
const float r4 = 0.17296; // 0.129115
int theta2_0 = 0;
float L1 = 0.3;    //
float L2 = r4 * 2; // 0.15942
float theta2_max = 160 * deg2rad;
*/
const float r1 = 0.0355;
const float r2 = 0.1223;
const float r3 = 0.1256;  
const float r4 = 0.1305; 
int theta2_0 = 0;
float L1 = 0.185;    
float L2 = 0.1625; 
float theta2_max = 160 * deg2rad;


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

// z-axis and suction
int vacuum = 5; // PCF
int cylinder = 11; // or 13 // make this the DB pin or one of the i2c pins from the extender

// camera variable
float camX[100];
float camY[100];
float receivedX, receivedY;
char receivedBuffer[32];
volatile bool newData = false;
volatile bool allPointsReceived = false;

// encoder parameters
volatile long position1 = 0;  // position for theta1
volatile long position2 = 0;  // position for theta2
const int totalCounts = 9600; // in encoder counts

// Limits in encoder counts for each motor
// motor 1
const int minPos1 = -(int)((float)totalCounts * (23.0 / 360.0)); // 23 degrees below parallel with x axis, negative because of the way we set up the encoders and directions
const int maxPos1 = (180.0 / 360.0) * (float)totalCounts;
// motor 2
const int minPos2 = 0; // (10.0 / 360.0) * (float)totalCounts;
const int maxPos2 = (180.0 / 360.0) * (float)totalCounts;


// Home Positions - CHANGE THESE TO ACTUAL X AND Y AFTER HOMING
float homePosX = 0.0; // add the link lengths together
float homePosY = 0.5; // 0

// PID PARAMETERS //

float Kp = 0.20, Ki = 0.0, Kd = 0.04; // Kp = 0.18, Kd = 0.006

// motor 1
float Input1, Output1, Setpoint1;
QuickPID PID1(&Input1, &Output1, &Setpoint1);

// motor 2
float Input2, Output2, Setpoint2;
QuickPID PID2(&Input2, &Output2, &Setpoint2);


// speed scaling factor for PID output, between 0 and 1
// this just limits the maximum speed
float speedScale = 0.8;
// I can also change pointDensity to affect smoothness/speed

// Serial
int rx_x, rx_y, rx_a, rx_s, rx_h, rx_d;

enum MachineStates
{
  Waiting,
  Home,
  Demo,
  Moving
};

MachineStates currentState = Waiting;


void setup()
{

  // SERIAL INITIALIZATION //
  Serial.begin(115200);
  // while (!Serial);
  delay(1000);
  Serial2.begin(115200, SERIAL_8N1, 10, 12); // rx, tx
  Serial.println("System Online. Listening for other ESP32 on Pins 10/12...");

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
  pcf.pinMode(vacuum, OUTPUT);
  pinMode(cylinder, OUTPUT);

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
  PID1.SetTunings(Kp, Ki, Kd);
  PID1.SetMode(PID1.Control::automatic);
  PID1.SetAntiWindupMode(PID1.iAwMode::iAwCondition);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTimeUs(5000);
  // motor2
  Input2 = position2;
  PID2.SetTunings(Kp, Ki, Kd);
  PID2.SetMode(PID2.Control::automatic);
  PID2.SetAntiWindupMode(PID2.iAwMode::iAwCondition);
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTimeUs(5000);
}

void loop()
{

  /*
  MotorDirection(1, HIGH);
  MotorDirection(2, HIGH);
  analogWrite(motor1PWM, 80);
  analogWrite(motor2PWM, 80);
  vTaskDelay(pdMS_TO_TICKS(500));
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  while (true);
  */

  // DEMO LOOP TESTING CODE
  // this allows us to bypass receiving serial array from the pi
  
  receivedPoints = 1;
  allPointsReceived = true;
  camX[0] = 0.3;
  camY[0] = 0.3;

  // theta 2 testing code
  // hit theta2 limit then go to 3 different PID controlled points
  /*
  bool limitHit = digitalRead(limit2);
  MotorDirection(2,LOW);
  analogWrite(motor2PWM, 80);
  while (!limitHit) {
    limitHit = digitalRead(limit2);
  }
  analogWrite(motor2PWM, 0);
  position2 = 0;
  pcf.digitalWrite(vacuum, HIGH);

  while (!PositionChange2(2000))
  {
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  while (true);
  
  vTaskDelay(pdMS_TO_TICKS(1000));

  PositionChange1(1000);
  PositionChange2(3000);

  // try getting rid of integral gain because dithering happens on this call of positionchange2
  while(!PositionChange1(1000) | !PositionChange2(3000))
  {
    PositionChange1(1000);
    PositionChange2(3000);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  while (true);

  */

  /* THETA 1 TESTING CODE 
  position1 = 0;  

  while (true) {
    suction(1);
    vTaskDelay(pdMS_TO_TICKS(200));
    suction(0);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  
  while (true);
  */


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
      xSemaphoreGive(dataMutex);

      // Logic to switch states
      if (rx_h == 1)
      {
        currentState = Home;
      }
      else if (rx_d == 1)
      {
        currentState = Demo;
      }
      else if (last_rx_x != rx_x || last_rx_y != rx_y || last_rx_a != rx_a || last_rx_s != rx_s)
      {
        updatePosition(rx_x, rx_y);
        last_rx_x = rx_x;
        last_rx_y = rx_y;
        last_rx_a = rx_a;
        last_rx_s = rx_s;
        Serial.print("Received X: ");
        Serial.print(rx_x);
        Serial.print(" Y: ");
        Serial.print(rx_y);
        currentState = Moving;
      }
    }

    break;

  case Home:
    HomeMotors();

    // find the actual starting positions after being homed, the insert here
    // currentPosX = homePosX;
    // currentPosY = homePosY;
    currentState = Waiting;
    break;

  case Demo:

    // We need to tell the robot to move out of the way of the mechanism so that the camera can take a good pic
    // THEN, the esp32 should send a signal through Serial2 giving the camera a green light  
    // At this point, the mechanism should be homed

    // reset demo variable
   /*
    while(!PositionChange1(pictureCount))
    {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    */
    Serial2.println("picture");

    // create some sort of statement that guarantees all data is sent, aka camX[receivedPoints] = 0
    if (!allPointsReceived)
    {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    else if (allPointsReceived)
    {
      const TickType_t actuationDelay = pdMS_TO_TICKS(500);
      for (int i = 0; i < receivedPoints; i++)
      {
        // move from current position to chip position and pick up
        MoveTo(camX[i], camY[i]);
        vTaskDelay(actuationDelay);
        zAxis(1);
        vTaskDelay(actuationDelay);
        suction(1);
        vTaskDelay(actuationDelay);
        zAxis(0);

        vTaskDelay(pdMS_TO_TICKS(3000));

        // move to carriage and drop off
        MoveTo(carriageX, carriageY);
        vTaskDelay(actuationDelay);
        zAxis(1);
        vTaskDelay(actuationDelay);
        suction(0);
        vTaskDelay(actuationDelay);
        zAxis(0);

        vTaskDelay(pdMS_TO_TICKS(3000));
      }
      receivedPoints = 0;
      allPointsReceived = false;
      //currentState = Waiting; uncomment after done with testing
      while (true);
    }
    break;

  case Moving:
    // 1. look at the latest data from Core 0 EVERY loop cycle
    if (xSemaphoreTake(dataMutex, (TickType_t)0) == pdTRUE)
    {
      rx_x = latestData.x;
      rx_y = latestData.y;
      rx_a = latestData.a;
      rx_s = latestData.s;
      xSemaphoreGive(dataMutex);
    }

    // 2. Update the target in real-time, which ignores previous target then
    updatePosition(rx_x, rx_y);
    inverseCalc(currentPosX, currentPosY, 1);

    Setpoint1 = constrain(radToPos(theta1[1]), minPos1, maxPos1);
    Setpoint2 = constrain(radToPos(THETA2[1]), minPos2, maxPos2);

    // 3. Drive motors
    bool arrive1 = PositionChange1(Setpoint1);
    bool arrive2 = PositionChange2(Setpoint2);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 4. Update pneumatics live
    zAxis(rx_a);
    suction(rx_s);

    // 5. Only exit when the robot has caught up to the target

    if (arrive1 && arrive2) 
    {
        currentState = Waiting; 
    }

    /*
    // printing deugging every 200 ms
    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime > 200)
    { // Only print 10 times per second
      Serial2.print("T1: ");
      Serial2.print(theta1[1]);
      Serial2.print(" T2: ");
      Serial2.print(THETA2[1]);
      Serial2.print(" X: ");
      Serial2.print(currentPosX);
      Serial2.print(" Y: ");
      Serial2.println(currentPosY);
      Serial2.print("M1 CS: ");
      Serial2.print(analogRead(motor1CS));
      Serial2.print(" M2 CS: ");
      Serial2.println(analogRead(motor2CS));
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
  analogWrite(motor2PWM, 50);
  bool limitHit1 = digitalRead(limit1);
  MotorDirection(1, LOW);
  analogWrite(motor1PWM, 80);
  while (!limitHit1) {
    limitHit1 = digitalRead(limit1);
  }
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  position1 = (int)(-(float)totalCounts * (23.0/360.0)); // 23 degrees below parallel with x axis
  
  // home theta 2
  limitHit2 = digitalRead(limit2);
  MotorDirection(2, LOW);
  analogWrite(motor2PWM, 80);
  while (!limitHit2) {
    limitHit2 = digitalRead(limit2);
  }
  analogWrite(motor2PWM, 0);
  position2 = (int)((float)totalCounts * (7.0/360.0)); // 7 degrees from parallel with theta1

  float start_theta1 = ((float)position1 / (float)totalCounts) * 2 * PI;
  float start_theta2 = ((float)position2 / (float)totalCounts) * 2 * PI;

  // set home position. ForwardCalc redefince Xcalc and Ycalc globals
  ForwardCalc(start_theta1, start_theta2);
  homePosX = Xcalc;
  homePosY = Ycalc;
  currentPosX = homePosX;
  currentPosY = homePosY;

  rx_h = 0;

  // PID testing
  /*
  vTaskDelay(pdMS_TO_TICKS(1000));

  while (!PositionChange2(1000)){
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  vTaskDelay(pdMS_TO_TICKS(1000));

  while(!PositionChange1(2000) | !PositionChange2(5000))
  {
    PositionChange1(2000);
    PositionChange2(5000);
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  vTaskDelay(pdMS_TO_TICKS(1000));

  syncCurrentPosition();
  MoveTo(0.0, 0.4);

  while (true);
  */
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
    digitalWrite(cylinder, HIGH);
    // Serial.println("Actuator On");
  }
  else if (toggleA == 0)
  {
    digitalWrite(cylinder, LOW);
    // Serial.println("Actuator Off");
  }
}

void suction(int toggleS)
{

  if (toggleS == 1)
  {
    pcf.digitalWrite(vacuum, HIGH);
    // Serial.println("Suction On");
  }
  else if (toggleS == 0)
  {
    pcf.digitalWrite(vacuum, LOW);
    // Serial.println("Suction Off");
  }
}

void updatePosition(int x, int y)
{
  // translate the positions rx_x and rx_y into actual positions in meters
  // we need a scale factor here, rx_x and rx_y are just integers
  currentPosX = homePosX + x * 0.01;
  currentPosY = homePosY + y * 0.01;
  
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

void receiveEvent(int howMany)
{
  if (currentState == Demo)
  {
    int i = 0;
    while (Wire.available() && i < (sizeof(receivedBuffer) - 1))
    {
      receivedBuffer[i] = Wire.read();
      i++;
    }
    receivedBuffer[i] = '\0'; // Null-terminate the string so sscanf knows where it ends
    newData = true;
  }
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
    Serial.println("Unachievable position, restart program");
    while (1)
      ;
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
  u2 = (B2 - sqrt(sq(A2) + sq(B2) - sq(C2))) / (C2 + A2);

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

// Background task to handle serial communication and i2c data
void CommTask(void *pvParameters)
{
  // i need to add the i2c receving code here and add inside for(;;)
  // i2c should send x and y positions until 0,0 is sent
  //

  String packetBuffer = "";

  for (;;)
  { // Infinite loop for the task
    while (Serial2.available() > 0)
    {
      char c = Serial2.read();

      if (c == '\n')
      {
        int tx, ty, ta, ts, th, td;
        int matched = sscanf(packetBuffer.c_str(), "X%d Y%d A%d S%d H%d D%d",
                             &tx, &ty, &ta, &ts, &th, &td);

        if (matched == 6)
        {
          // Update the shared struct safely using a Mutex
          if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE)
          {
            latestData.x = tx;
            latestData.y = ty;
            latestData.a = ta;
            latestData.s = ts;
            latestData.h = th;
            latestData.d = td;

            /* debugging print statement
            static uint32_t lastTime = millis();
            if (millis() - lastTime > 500)
            {
              Serial2.print("Received X: ");
              Serial2.print(tx);
              Serial2.print(" Y: ");
              Serial2.print(ty);
              Serial2.print(" A: ");
              Serial2.print(ta);
              Serial2.print(" S: ");
              Serial2.print(ts);
              Serial2.print(" H: ");
              Serial2.print(th);
              Serial2.print(" D: ");
              Serial2.println(td);
              lastTime = millis();
            }
            */
            
            // white on edge to receive from screen, green on edge to receive from esp32 
            xSemaphoreGive(dataMutex);
          }
        }

        if (matched == 2){
          // This is for the demo mode, where we just receive x and y coordinates until 0,0 is sent
          if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE)
          {
            latestData.x = tx;
            latestData.y = ty;

            // as each x and y point is sent, we need to add it to camX[i] an camY[i]
            if (receivedPoints < 100) // Assuming a maximum of 100 points
            {
              camX[receivedPoints] = tx;
              camY[receivedPoints] = ty;
              receivedPoints++;
            }


            static uint32_t lastTime = millis();
            if (millis() - lastTime > 500)
            {
              Serial2.print("Received X: ");
              Serial2.print(tx);
              Serial2.print(" Y: ");
              Serial2.println(ty);
              lastTime = millis();
            }

            if (tx == 0 && ty == 0)
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

    vTaskDelay(5 / portTICK_PERIOD_MS);
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
  vTaskDelay(pdMS_TO_TICKS(100)); // small delay to ensure current position is updated before calculating trajectory
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
    bool arrived1 = false;
    bool arrived2 = false;
    //use this loop instead of plain positionchange for more accuracy
    int timeout = 0; 
    while (!PositionChange1(Setpoint1) | !PositionChange2(Setpoint2))
    {
      //arrived1 = PositionChange1(Setpoint1);
      //arrived2 = PositionChange2(Setpoint2);
      vTaskDelay(pdMS_TO_TICKS(5));
      timeout++;
      if (timeout > 500) break;
    }
    //PositionChange1(Setpoint1);
    //PositionChange2(Setpoint2);
    
    // intentional delay between points
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  while (!PositionChange1(Setpoint1) || !PositionChange2(Setpoint2)) {
    vTaskDelay(TICK_5MS);
  }

}