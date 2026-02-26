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

// I2C Address for raspberry pi
#define I2C_ADDR 0x08 // might change

// Mutex and Multithreading
SemaphoreHandle_t dataMutex;
struct ControlData
{
  int x, y, a, s, h, d;
} latestData;
void CommTask(void *pvParameters);

// VARIABLE DEFINITIONS //

const float deg2rad = PI / 180;
const float rad2deg = 180 / PI;

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
float carriageY = 0.0;
int receivedPoints = 0;

// Position
volatile float currentPosX = 0.0;
volatile float currentPosY = 0.5;
int last_rx_x = -999;
int last_rx_y = -999;
int last_rx_a = 0;
int last_rx_s = 0;

// link lengths all in mm
// the commented values are the link lengths of nicky's model
// CALIBRATE R1 ONCE ASSEMBLED
const float r1 = 0.04717; // 0.;
const float r2 = 0.16117; // 0.123;
const float r3 = 0.1651;  // 0.126;
const float r4 = 0.17296; // 0.129115
int theta2_0 = 0;
float L1 = 0.3;    //
float L2 = r4 * 2; // 0.15942
float theta2_max = 160 * deg2rad;

/*
Homing switches (normally LOW): GPIO01, GPIO2

High current switch (vacuum motor on/off): GPIO38

Motor Driver A:
INA - GPIO4 X
ENA - GPIO5 X
PWM - GPIO6 X
CS - GPIO7
ENB - GPIO15 X
INB - GPIO16 X

Motor Driver B:
INA - GPIO17 X
ENA - GPIO8 X
PWM - GPIO3 X
CS - GPIO9
ENB - GPIO10 X
INB - GPIO11 X
*/

// motor1 - the motor that controls theta1
int motor1PWM = 6;
int motor1dirA = 4; // HIGH always increases position1, if black top and red bottom then moves CCW
int motor1dirB = 16;
int motor1encA = 13;
int motor1encB = 14;
int motor1enableA = 5;
int motor1enableB = 15;

// motor2 - the motor that controls theta2
int motor2PWM = 3;
int motor2dirA = 17; // HIGH always increases position2, if red top and black bottom then moves CCW
int motor2dirB = 11;
int motor2encA = 47;
int motor2encB = 48;
int motor2enableA = 8;
int motor2enableB = 10;

// Limit switches
int limit1 = 1; // this limit is for theta1
int limit2 = 2; // this limit is for theta2

// z-axis and suction
int vacuum = 38;
int cylinder = 35; // ?

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
const int minPos1 = 0;
const int maxPos1 = (180.0 / 360.0) * (float)totalCounts;
// motor 2
const int minPos2 = 0; // (10.0 / 360.0) * (float)totalCounts;
const int maxPos2 = (180.0 / 360.0) * (float)totalCounts;

// Home Positions - CHANGE THESE TO ACTUAL X AND Y AFTER HOMING
float homePosX = 0.0; // add the link lengths together
float homePosY = 0.5; // 0

// PID PARAMETERS //

float Kp = 0.18, Ki = 0.0, Kd = 0.006;

// motor 1
float Input1, Output1, Setpoint1;
QuickPID PID1(&Input1, &Output1, &Setpoint1);

// motor 2
float Input2, Output2, Setpoint2;
QuickPID PID2(&Input2, &Output2, &Setpoint2);

// speed scaling factor for PID output, between 0 and 1
float speedScale = 0.8;

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
void MoveTo(float startX, float startY, float endX, float endY);

void setup()
{

  // SERIAL INITIALIZATION //
  Serial.begin(115200);
  // while (!Serial);
  delay(500);
  Serial2.begin(115200, SERIAL_8N1, 36, 37); // rx, tx
  Serial.println("System Online. Listening for other ESP32 on Pins 38/39...");

  // I2C initialization
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);

  // PIN INITIALIZATION //

  // motor 1
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1dirA, OUTPUT);
  pinMode(motor1dirB, OUTPUT);
  pinMode(motor1encA, INPUT_PULLDOWN);
  pinMode(motor1encB, INPUT_PULLDOWN);

  pinMode(motor1enableA, OUTPUT);
  pinMode(motor1enableB, OUTPUT);
  digitalWrite(motor1enableA, HIGH);
  digitalWrite(motor1enableB, HIGH);
  digitalWrite(motor1dirA, HIGH);
  digitalWrite(motor1dirB, LOW);

  // motor 2
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2dirA, OUTPUT);
  pinMode(motor2dirB, OUTPUT);
  pinMode(motor2encA, INPUT_PULLDOWN);
  pinMode(motor2encB, INPUT_PULLDOWN);

  pinMode(motor2enableA, OUTPUT);
  pinMode(motor2enableB, OUTPUT);
  digitalWrite(motor2enableA, HIGH);
  digitalWrite(motor2enableB, HIGH);
  digitalWrite(motor2dirA, HIGH);
  digitalWrite(motor2dirB, LOW);

  // limit switches
  pinMode(limit1, INPUT_PULLDOWN);
  pinMode(limit2, INPUT_PULLDOWN);

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
  // motor2
  Input2 = position2;
  PID2.SetTunings(Kp, Ki, Kd);
  PID2.SetMode(PID2.Control::automatic);
  PID2.SetAntiWindupMode(PID2.iAwMode::iAwCondition);
  PID2.SetOutputLimits(-255, 255);
}

void loop()
{

  /* Debugging statement
  while (true) {
    MotorDirection(1, LOW);
    analogWrite(motor1PWM, 20);
    Serial.print("Position 1: ");
    Serial.print(position1);
    Serial.print(" | Position 2: ");
    Serial.print(position2);
    Serial.print(" | X: ");
    Serial.print(latestData.x);
    Serial.print(" | Y: ");
    Serial.println(latestData.y);
    delay(1000);
  }
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

    // create some sort of statement that guarantees all data is sent, aka camX[receivedPoints] = 0
    // we will be using the batch method, send all data points, then move
    if (!allPointsReceived)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else
    {
      for (int i = 0; i < receivedPoints; i++)
      {

        // move from current position to chip position and pick up
        MoveTo(currentPosX, currentPosY, camX[i], camY[i]);
        currentPosX = camX[i];
        currentPosY = camY[i];
        delay(50);
        zAxis(1);
        delay(50);
        suction(1);
        delay(50);
        zAxis(0);

        // move to carriage and drop off
        MoveTo(currentPosX, currentPosY, carriageX, carriageY);
        currentPosX = carriageX;
        currentPosY = carriageY;
        delay(50);
        zAxis(1);
        delay(50);
        suction(0);
        delay(50);
        zAxis(0);
      }
      receivedPoints = 0;
      allPointsReceived = false;
      currentState = Waiting;
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

    // 3. Drive motors (NON-BLOCKING)
    bool arrive1 = PositionChange1(Setpoint1);
    bool arrive2 = PositionChange2(Setpoint2);

    // 4. Update pneumatics live
    zAxis(rx_a);
    suction(rx_s);

    // 5. Only exit when the robot has caught up to the target
    while (!arrive1 || !arrive2)
    {
      arrive1 = PositionChange1(Setpoint1);
      arrive2 = PositionChange2(Setpoint2);
      zAxis(rx_a);
      suction(rx_s);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    static uint32_t lastPrintTime = 0;

    if (millis() - lastPrintTime > 200)
    { // Only print 10 times per second
      Serial.print("T1: ");
      Serial.print(theta1[1]);
      Serial.print(" T2: ");
      Serial.print(THETA2[1]);
      Serial.print(" X: ");
      Serial.print(currentPosX);
      Serial.print(" Y: ");
      Serial.println(currentPosY);
      lastPrintTime = millis();
    }
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
      digitalWrite(motor1dirA, HIGH);
      digitalWrite(motor1dirB, LOW);
    }
    else
    {
      digitalWrite(motor1dirA, LOW);
      digitalWrite(motor1dirB, HIGH);
    }
  }
  else if (motor == 2)
  {
    if (direction == HIGH)
    {
      digitalWrite(motor2dirA, HIGH);
      digitalWrite(motor2dirB, LOW);
    }
    else
    {
      digitalWrite(motor2dirA, LOW);
      digitalWrite(motor2dirB, HIGH);
    }
  }
}

void HomeMotors()
{

  /*Serial.println("Homing... ");

  suction(LOW);
  zAxis(LOW);

  MotorDirection(1, LOW);  // CW
  MotorDirection(2, LOW);  // CW

  while (!digitalRead(limit1)) {
    analogWrite(motor1PWM, 80);
  }
  analogWrite(motor1PWM, 0);
  position1 = 0;

  while (!digitalRead(limit2)) {
    analogWrite(motor2PWM, 80);
  }
  analogWrite(motor2PWM, 0);
  position2 = 0;*/

  Serial.println("Homing Disabled, re-enable for proper function");
  position1 = 0;
  position2 = 0;
  rx_h = 0;
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
    digitalWrite(vacuum, HIGH);
    // Serial.println("Suction On");
  }
  else if (toggleS == 0)
  {
    digitalWrite(vacuum, LOW);
    // Serial.println("Suction Off");
  }
}

void updatePosition(int x, int y)
{
  // translate the positions rx_x and rx_y into actual positions in meters
  // we need a scale factor here, rx_x and rx_y are just integers
  currentPosX = homePosX + x * 0.01;
  currentPosY = homePosY + y * 0.01;
  // delay(100);
}

bool PositionChange1(int target)
{

  // check for serial data here, then go back to waiting
  Setpoint1 = target;
  Input1 = position1;
  PID1.Compute();
  MotorDirection(1, Output1 >= 0 ? HIGH : LOW);

  /* stops the motor based on the bounds
  if (position1 < minPos1) {
    Serial.println("Motor 1 Minimum Position Exceeded");
    Output1 = 0;
    analogWrite(motor1PWM, 0);
    return true;
  } else if (position1 > maxPos1) {
    Serial.println("Motor 1 Maximum Position Exceeded");
    Output1 = 0;
    analogWrite(motor1PWM, 0);
    return true;
  }
  */
  Output1 = Output1 * speedScale;
  analogWrite(motor1PWM, abs(Output1));

  if (abs(position1 - target) <= 5)
  {
    analogWrite(motor1PWM, 0);
    Output1 = 0;
    // PrintStats1();
    return true;
  }

  if (Output1 != 0 && abs(Output1) < 19)
  { // THIS DEADZONE WILL NEED TO BE ADJUSTED (after physical mechanism updated)
    Output1 = (Output1 >= 0) ? 15 : -15;
  }
  // PrintStats1();
  return false;
}

bool PositionChange2(int target)
{
  Setpoint2 = target;
  Input2 = position2;
  PID2.Compute();
  MotorDirection(2, Output2 >= 0 ? HIGH : LOW);

  /* stops motor based on the bounds
  if (position2 < minPos2) {
    Serial.println("Motor 2 Minimum Position Exceeded");
    Output2 = 0;
    analogWrite(motor2PWM, 0);
    return true;
  } else if (position2 > maxPos2) {
    Serial.println("Motor 2 Maximum Position Exceeded");
    Output2 = 0;
    analogWrite(motor2PWM, 0);
    return true;
  }
  */
  Output2 = Output2 * speedScale;
  analogWrite(motor2PWM, abs(Output2));

  if (abs(position2 - target) <= 0)
  {
    analogWrite(motor2PWM, 0);
    Output2 = 0;
    // PrintStats2();
    return true;
  }

  if (Output2 != 0 && abs(Output2) < 19)
  { // THIS DEADZONE WILL NEED TO BE ADJUSTED (after physical mechanism updated)
    Output2 = (Output2 >= 0) ? 15 : -15;
  }
  // PrintStats2();
  return false;
}

void countChangeA1()
{
  if (digitalRead(motor1encB) != digitalRead(motor1encA))
    position1++;
  else
    position1--;
}

void countChangeB1()
{
  if (digitalRead(motor1encB) == digitalRead(motor1encA))
    position1++;
  else
    position1--;
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
  Serial.print(position2);
}

void Stop()
{
  analogWrite(motor1PWM, LOW);
  analogWrite(motor2PWM, LOW);
  /*
  digitalWrite(vacuum, LOW);
  delay(200);
  digitalWrite(cylinder, LOW);
  */
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

  /* for reference and calc checks
  const float r1 = 0.04717;
  const float r2 = 0.16117;
  const float r3 = 0.1651;
  const float r4 = 0.17296;
  int theta2_0 = 0;
  float L1 = 0.3;
  float L2 = r4 * 2;
  */

  // Slot Translational Distance w/ Solution Selection
  if (R <= (L1 + r1) + L2)
  {
    d = 0;
  }
  else
  {
    d = -(L1 + r1) * cos(theta2_0) - L2 + sqrt(sq(R) - sq(sin(theta2_0)) * sq(L1 + r1));
  }

  // Solve for Theta2 according to d
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

  /*
  if (THETA2[i] < 0) {
    THETA2[i] += 2 * PI;
  }
  */
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

    

    if (newData)
    {
      int parsed = sscanf(receivedBuffer, "X%d Y%d", &receivedX, &receivedY);

      if (parsed == 2)
      { // sscanf returns how many variables it successfully filled
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
          if (receivedX == 0 && receivedY == 0)
          {
            Serial.println("No more points");
            newData = false;
            allPointsReceived = true;
          }
          else if (receivedPoints < 100)
          {
            Serial.print("Parsed successfully! X: ");
            Serial.print(receivedX);
            Serial.print(" | Y: ");
            Serial.println(receivedY);
            camX[receivedPoints] = receivedX;
            camY[receivedPoints] = receivedY;
            receivedPoints++;
          } else {
            Serial.println("Error: Too many points received, max is 100.");
          }
          xSemaphoreGive(dataMutex);
        }
      }
      else
      {
        Serial.println("Error: Message format was wrong.");
      }
      newData = false; // Reset the flag for the next message
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
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
void MoveTo(float startX, float startY, float endX, float endY)
{
  dist = sqrt(pow((endX - startX), 2) + pow((endY - startY), 2));
  pointCount = int(dist * pointDensity);
  linspace(startX, endX, pointCount, xPoints);
  linspace(startY, endY, pointCount, yPoints);
  for (int i = 0; i < pointCount; i++)
  {
    inverseCalc(xPoints[i], yPoints[i], i);
    Setpoint1 = constrain(radToPos(theta1[i]), minPos1, maxPos1);
    Setpoint2 = constrain(radToPos(THETA2[i]), minPos2, maxPos2);
    bool arrived1 = PositionChange1(Setpoint1);
    bool arrived2 = PositionChange2(Setpoint2);
    while (!arrived1 || !arrived2)
    {
      arrived1 = PositionChange1(Setpoint1);
      arrived2 = PositionChange2(Setpoint2);
      vTaskDelay(5 / portTICK_PERIOD_MS); // Short delay to prevent CPU hogging
    }
  }
}