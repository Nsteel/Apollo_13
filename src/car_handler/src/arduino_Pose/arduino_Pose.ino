/*
 * arduino_Pose.ino
 *
 *      Authors: Sebastian Ehmes
 *         Nicolas Acero
 *         Huynh-Tan Truong
 *         Li Zhao
 *         
 *      Co-Author: Jeff Rowberg   
 */
 * Arduino Code to read, calculate and send position and orientation
*/
#include <Wire.h>
#include <MP
U6050.h>
#include <Encoder.h>

//MPU Sensor
MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll,Yaw and angular velocity values
float pitch = 0;
float roll = 0;
float yaw = 0;
float w = 0;

// Connection pins of the encoder
const int directionPin = 6;
const int clkPin =  7;

// Enconder
Encoder myEnc(directionPin, clkPin);

// Radians between two encoder ticks
const float radPerTick = 0.1047;
// Radious of the car wheel
const float wheelRadious = 0.032;

long oldPosition  = -999;

//travelled distance value
float s = 0;


void setup() 
{
  Serial.begin(115200);
  pinMode(directionPin, INPUT);
  pinMode(clkPin, INPUT);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);
}

// Read the current encoder ticks and return the current travelled distance
void readDistance() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  }
  // calculation of travelled distance
  s = oldPosition * radPerTick * tireRadious;
}

void loop()
{
  timer = millis();
  readDistance();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();
  // Calculate Pitch, Roll, Yaw and angular velocity
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
  Vector rawGyro = mpu.readRawGyro();
  w = rawGyro.ZAxis;
  // Angular velocity in Radians!
  w = w * M_PI/180;
  //  According to the sensor datasheet, this multiplication is needed to obtain the angular velocity
  w = w * 25/2000;

  /* The following code part sends a line with the data over the serial connection.
     The format of the information line is the following: s*w*yaw*timer*checksum*
     The information is separate with the character '*'. 
     On the other hand a checksum is also sent in order to avoid transmission problems. 
   */
  
  Serial.print(s);
  Serial.print("*");
  Serial.print(w);
  Serial.print("*");
  Serial.print(yaw * M_PI/180);
  Serial.print("*");
  Serial.print(timer);
  Serial.print("*");
  Serial.print(s+w*25/2000+yaw * M_PI/180+timer);
  Serial.println("*");

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}


