/*
 * arduino_Pose.ino
 *
 *      Authors: Sebastian Ehmes
 *         Nicolas Acero
 *         Huynh-Tan Truong
 *         Li Zhao
 *         
 *      Co-Author: Jeff Rowberg   
 *
 * Arduino Code to read, calculate and send position and orientation
*/
#include <Wire.h>
#include <MPU6050.h>
#include <Encoder.h>
#include <HMC5883L.h>

//MPU Sensor
MPU6050 mpu;
//Magnetometer
HMC5883L compass;

// Timers
unsigned long timer = 0;
const float timeStep = 0.01;

//factors
const float degToRad = M_PI/180;

//MPU Pitch, Roll,Yaw and angular velocity values
Vector normW;
Vector rawGyro;
float pitch = 0;
float roll = 0;
float yaw = 0;

float w_x = 0;
float w_y = 0;
float w_z = 0;

Vector g;
float sr, cr, sp, cp, sy, cy;
Vector offset;
Vector normA;
float ax = 0;
float ay = 0;
float az = 0;

//MAG Values
float declinationAngle = 0;
Vector mag;
float heading = 0;
float m_x = 0;
float m_y = 0;
float m_z = 0;

// Connection pins of the encoder
const int directionPin = 6;
const int clkPin =  7;

// Enconder
Encoder myEnc(directionPin, clkPin);

// Radians between two encoder ticks
const float radPerTick = 0.1047;
// Radious of the car wheel
const float wheelRadius = 0.032;

long oldPosition  = -999;

//travelled distance value
float s = 0;

void calcGravity(){
  sr = sin(roll);
  cr = cos(roll);
  sp = sin(pitch);
  cp = cos(pitch);
  sy = sin(yaw);
  cy = cos(yaw);

  g.XAxis = -9.81*(sr*sy+cr*sp*cy);
  g.YAxis = -9.81*(-sr*cy+cr*sp*sy);
  g.ZAxis = 9.81*(cr*cp);
}

void calcRPY(Vector * w_xyz){
  roll = roll + w_xyz->XAxis*degToRad * timeStep;
  pitch = pitch + w_xyz->YAxis*degToRad * timeStep;
  yaw = yaw + w_xyz->ZAxis*degToRad * timeStep;
}

void configAccelOffsets(){
  /*
  Serial.println("Calculating Gravity-Vector: ...");
  delay(200);
  */
  int c = 0;
  while(c<10){
    delay(timeStep*1000);
    normW = mpu.readNormalizeGyro();
    calcRPY(&normW);
    c++;
  }
  calcGravity();
  /*
  Serial.print("Gravity-Vector: [");
  Serial.print(g.XAxis);
  Serial.print(", ");
  Serial.print(g.YAxis);
  Serial.print(", ");
  Serial.print(g.ZAxis);
  Serial.println("]");
  delay(200);


  Serial.println("Calculating Offsets without Gravity-Effects: ...");
  delay(200);
  */
  offset.XAxis = 0;
  offset.YAxis = 0;
  offset.ZAxis = 0;

  c=0;
  while(c<100){
    delay(10);
    normA = mpu.readNormalizeAccel();
    offset.XAxis += (normA.XAxis-g.XAxis);
    offset.YAxis += (normA.YAxis-g.YAxis);
    offset.ZAxis += (normA.ZAxis-g.ZAxis);
    c++;
  }
  
  offset.XAxis/=c;
  offset.YAxis/=c;
  offset.ZAxis/=c;
  /*
  Serial.print("Offset-Vector: [");
  Serial.print(offset.XAxis);
  Serial.print(", ");
  Serial.print(offset.YAxis);
  Serial.print(", ");
  Serial.print(offset.ZAxis);
  Serial.println("]");
  delay(200);
  */
  
}

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
  
  // Enable MPU bypass mode
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);
  
  // Initialize Initialize HMC5883L
 /* while (!compass.begin())
  {
    delay(500);
  }*/
  // <--Set compass begin:
  // Set measurement range
  //compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  //compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  //compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  //compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  //compass.setOffset(0, 0); 
  // Calculate declination angle
  //declinationAngle = (2.0 + (11.0 / 60.0)) / (180 / M_PI);
  // Set compass end !->
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  //mpu.setAccelOffsetX(-936);
  //mpu.setAccelOffsetY(260);
  //mpu.setAccelOffsetX(1653);
  //mpu.setAccelOffsetX(0);
  //mpu.setAccelOffsetY(0);
  //mpu.setAccelOffsetX(0);
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0);
  configAccelOffsets();
  
}

// Read the current encoder ticks and return the current travelled distance
void readDistance() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  }
  // calculation of travelled distance
  s = oldPosition * radPerTick * wheelRadius;
}

// Correct angle
float correctAngle(float heading) {
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
}

// No tilt compensation
float noTiltCompensate(Vector mag) {
  return atan2(mag.YAxis, mag.XAxis);
}

void buildSensorMsg(String* outstring){
   /* The following code part sends a line with the data over the serial connection.
     The format of the information line is the following: #s**w_x*w_y*w_z*ax*ay*az*m_x*m_y*m_z*
     The information is separate with the character '*'. 
     On the other hand a checksum is also sent in order to avoid transmission problems. 
   */
  *outstring = "#";
  outstring->concat(s);
  outstring->concat("*");
  outstring->concat(w_x);
  outstring->concat("*");
  outstring->concat(w_y);
  outstring->concat("*");
  outstring->concat(w_z);
  outstring->concat("*");
  outstring->concat(ax);
  outstring->concat("*");
  outstring->concat(ay);
  outstring->concat("*");
  outstring->concat(az);
  outstring->concat("*");
  outstring->concat(m_x);
  outstring->concat("*");
  outstring->concat(m_y);
  outstring->concat("*");
  outstring->concat(m_z);
  outstring->concat("*");
  outstring->concat("\n");
}

void loop()
{
  timer = millis();
  readDistance();
  // Read normalized angular velocity values
  normW = mpu.readNormalizeGyro();
  // Read normalized acceleration values
  normA = mpu.readNormalizeAccel();
  // Read normalized magnetic field values
  //mag = compass.readNormalize();

  // angular velocity values -> to rad
  w_x = normW.XAxis*degToRad;
  w_y = normW.YAxis*degToRad;
  w_z = normW.ZAxis*degToRad;

  // remove offset from acceleration values
  ax = normA.XAxis-offset.XAxis;
  ay = normA.YAxis-offset.YAxis;
  az = normA.ZAxis-offset.ZAxis;

  m_x = mag.XAxis;
  m_y = mag.YAxis;
  m_z = mag.ZAxis;
  
  //sensor msg
  String outstring;
  buildSensorMsg(&outstring);
  Serial.print(outstring);
  
  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}


