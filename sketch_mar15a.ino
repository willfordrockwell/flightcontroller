#include <MPU6050_tockn.h>

#include <Wire.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 4
#define SCL_PIN 5
#endif

#define calculatedGyroOffsetX -6.65
#define calculatedGyroOffsetY -1.62
#define calculatedGyroOffsetZ -1.62

#define BAUD_RATE 19200

#define MAX_SIZE_OF_SERIAL 64

#define DEBUG false

#define ACCELEROMETER_COEF 0.1
#define GYRO_COEF 0.1

typedef enum
{
  X,
  Y,
  Z
} COORD;

int leftFront = 3, rightFront = 2, leftRear = 10, rightRear = 11;

MPU6050 mySensor(Wire, ACCELEROMETER_COEF, GYRO_COEF);
/*
  Trottle - высота  (Левый стик, вертикаль)
  Pitch - Тангаж    (Правый стик, вертикаль)
  Roll -  Крен      (Правый стик, горизонталь)
  Yaw -   Рысканье  (Левый стик, горизонталь)
*/

//delaytime
int dT = 2;

//PID's coefficients
float KpYaw =   19.04041, KiYaw =   -0.07566, KdYaw =  	36.17238; //YAW   РЫСКАНЬЕ - Z
float KpRoll =  15.20834, KiRoll =  -0.00005, KdRoll = 	38.19664; //ROLL  КРЕН     - X
float KpPitch = 23.07135, KiPitch = -0.01472, KdPitch = 44.36351; //PITCH ТАНГАЖ   - Y

//PID's values to storage
int PIDYaw = 0, PIDRoll = 0, PIDPitch = 0;

//Errors to PID
int errorYaw = 0, errorRoll = 0, errorPitch = 0;

//Last values of calculateErrors for derivation
float lastYaw = 0.0, lastRoll = 0.0, lastPitch = 0.0;

//Integrator values for inegration;
int integratedYaw = 0, integratedRoll = 0, integratedPitch = 0;

//Values for motors
int LF, RF, LR, RR;

//Input values from serial
int inTrottle = 0, inYaw = 0, inRoll = 0, inPitch = 0;

//Input raw values from MPU60560
float inAngleX = 0.0, inAngleY = 0.0, inAngleZ = 0.0;

void getRadFromDegrees()
{
	inAngleX *= 0.0174;
	inAngleY *= 0.0174;
	inAngleZ *= 0.0174;
}

void writeMotors()
{
  digitalWrite(leftFront,LF);
  digitalWrite(rightFront, RF);
  digitalWrite(leftRear, LR);
  digitalWrite(rightRear, RR);
}

void getAngles()
{
  inAngleX = mySensor.getAngleX();
  inAngleY = mySensor.getAngleY();
  inAngleZ = mySensor.getAngleZ();
}

void PIDs()
{
  PIDYaw = (int)(KpYaw * errorYaw + KdYaw * (errorYaw - lastYaw) / dT / 0.001 + KiYaw * (integratedYaw + errorYaw) * dT * 0.001);
  integratedYaw += errorYaw;
  lastYaw = errorYaw;

  PIDRoll = (int)(KpRoll * errorRoll + KdRoll * (errorRoll - lastRoll) / dT / 0.001 + KiRoll * (integratedRoll + errorRoll) * dT * 0.001);
  integratedRoll += errorRoll;
  lastRoll = errorRoll;

  PIDPitch = (int)(KpPitch * errorPitch + KdPitch * (errorPitch - lastPitch) / dT / 0.001 + KiPitch * (integratedPitch + errorPitch) * dT * 0.001);
  integratedPitch += errorPitch;
  lastPitch = errorPitch;
}

void calculateErrors()
{
  errorYaw = inYaw - inAngleZ;
  errorRoll = inRoll - inAngleX;
  errorPitch = inPitch - inAngleY;
}

void countMotors()
{
  LF = constrain(inTrottle - PIDPitch - PIDYaw + PIDRoll, 0, 255);
  RF = constrain(inTrottle - PIDPitch + PIDYaw - PIDRoll, 0, 255);
  LR = constrain(inTrottle + PIDPitch + PIDYaw + PIDRoll, 0, 255);
  RR = constrain(inTrottle + PIDPitch - PIDYaw - PIDRoll, 0, 255);
}

void readCommandFromSerial()
{
  float angle = 0.0;
  COORD inputCoord;
  char message[MAX_SIZE_OF_SERIAL];
  int i = 0;

  for (i = 0; i < MAX_SIZE_OF_SERIAL; i++) {
    message[i] = '\0';
  }

  i = 0;
  while (Serial.available() > 0 && i < MAX_SIZE_OF_SERIAL) {
    message[i] = Serial.read();
    i++;
  }
  if (i == 0)
    return;

  if (strncmp(message, "Z", 1) == 0 || strncmp(message, "z", 1) == 0)
    inputCoord = Z;
  else if (strncmp(message, "Y", 1) == 0 || strncmp(message, "y", 1) == 0)
    inputCoord = Y;
  else if (strncmp(message, "X", 1) == 0 || strncmp(message, "x", 1) == 0)
    inputCoord = X;
  else if (strncmp(message, "T", 1) == 0 || strncmp(message, "t", 1) == 0) {
    char *pch = strtok(message, " ");
    pch = strtok(NULL, " ");
    inTrottle = atoi(pch);
    return;
  }

  char *pch = strtok(message, " ");
  pch = strtok(NULL, " ");

  angle = atof(pch);

  switch (inputCoord)
  {
  case Z:
    inYaw = angle;
    break;
  case Y:
    inRoll = angle;
    break;
  case X:
    inPitch = angle;
    break;
  default:
   break;
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);
  // put your setup code here, to run once:
  // setup pins (digital: 2 in for accelerometer and gyro, 4 out for motors)
  pinMode(leftFront, OUTPUT);
  pinMode(rightFront, OUTPUT);
  pinMode(leftRear, OUTPUT);
  pinMode(rightRear, OUTPUT);

#ifdef _ESP32_HAL_I2C_H_        // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
#else
  Wire.begin();
#endif

  mySensor.begin();

  if (DEBUG)
    mySensor.calcGyroOffsets(true);
  else
  {
    mySensor.calcGyroOffsets();
    mySensor.setGyroOffsets(calculatedGyroOffsetX, calculatedGyroOffsetY, calculatedGyroOffsetZ);
  }
}

void debugOutput(const char *param)
{
  if (strcmp(param, "angle") == 0)
  {
    Serial.print("\nangleX:");
    Serial.print(inAngleX);

    Serial.print("\tangleY:");
    Serial.print(inAngleY);

    Serial.print("\tangleZ:");
    Serial.print(inAngleZ);
    return;
  }
}

void loop()
{
  //get input
  readCommandFromSerial();
  //get datas
  mySensor.update();
  getAngles();
  getRadFromDegrees();
  if (DEBUG){
    debugOutput("angle");
  }
  //calculate calculateErrors
  calculateErrors();
  //PID
  PIDs();
  //count motors
  countMotors();
  //write motors
  writeMotors();

  delay(dT);
}
