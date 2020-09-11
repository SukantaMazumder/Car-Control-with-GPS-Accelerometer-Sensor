
//library & variables for Accelarometer

#include <Wire.h>  // Wire library - used for I2C communication
int ADXL345 = 0x53; // The ADXL345 sensor I2C address
float X_out, Y_out, Z_out;  // Outputs
float roll, pitch, rollF, pitchF = 0;


//library & variables for GPS

// The serial connection to the GPS device
#include <SoftwareSerial.h>
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);
//GPS Library
#include <TinyGPS++.h>
TinyGPSPlus gps;
int num_sat, gps_speed;
String heading;

//variables for acutator
const int IN1 = 4;   //Left Actuator +
const int IN2 = 5;   //Left Actuator - 
const int IN3 = 6;   //Right Actuator + 
const int IN4 = 7;   //Rgiht Actuator - 


void setup()
{
  ss.begin(GPSBaud);
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // Bit D3 High for measuring enable (8dec -> 0000 1000 binary)
  Wire.endTransmission();
  delay(10);
  //Off-set Calibration
  //X-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1E);
  Wire.write(1);
  Wire.endTransmission();
  delay(10);
  //Y-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1F);
  Wire.write(-2);
  Wire.endTransmission();
  delay(10);
  //Z-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x20);
  Wire.write(-9);
  Wire.endTransmission();
  delay(10);

}



//everything is controlled here
void loop()
{
  Get_GPS(); //Get GPS data
  Angle_Measurement();

  //change the roll according to left, right , straight callibration.
  // here rollF value between 4  & -4 means  the car position is in straight

  if (gps_speed >= 60 && (rollF >= -4 && rollF <= 4))  //both actuator will extend
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(20000);  // callibrate the delay time untill the actuator fully extend. 1000ms  = 1 Sec
  }

  if (gps_speed < 60 && (rollF >= -4 && rollF <= 4)) //both actuator will be at retract position if moving straight at 60 Km/h or Greater
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(20000);  // callibrate the delay time untill the actuator fully extend. 1000ms  = 1 Sec
  }

  if (rollF <= -5)   // car turn left, left actuator will extend & right will retract
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(20000);  // callibrate the delay time untill the actuator fully extend or retract. 1000ms  = 1 Sec
  }

  if (rollF > 5)   // car turn right , right actuator will extend & left will retract
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(20000); // callibrate the delay time untill the actuator fully extend or retract. 1000ms  = 1 Sec
  }



  //uncomment for showing speed on serial monitor
  
  //Serial.println(gps_speed);

  //uncomment for callibration & decide which value you will use for detecting straight, right & left

  //Serial.print(rollF);
  //Serial.print("/");
  //Serial.println(pitchF);

}




// This custom version of delay() ensures that the gps object is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void Get_GPS()
{


  num_sat = gps.satellites.value();

  if (gps.location.isValid() == 1)
  {
    gps_speed = gps.speed.kmph();
    heading = gps.cardinal(gps.course.value());
  }
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected: check wiring.");
  }
}

void Angle_Measurement()
{
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read() | Wire.read() << 8); // X-axis value
  X_out = X_out / 256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Y_out = ( Wire.read() | Wire.read() << 8); // Y-axis value
  Y_out = Y_out / 256;
  Z_out = ( Wire.read() | Wire.read() << 8); // Z-axis value
  Z_out = Z_out / 256;

  // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  roll = atan(Y_out / sqrt(pow(X_out, 2) + pow(Z_out, 2))) * 180 / PI;
  pitch = atan(-1 * X_out / sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;

  // Low-pass filter
  rollF = 0.94 * rollF + 0.06 * roll;
  pitchF = 0.94 * pitchF + 0.06 * pitch;
}
