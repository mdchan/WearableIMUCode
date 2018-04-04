

/* Chan */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

bool goFlag = false;
int analogPin1 = 8;
int analogPin2 = 9;
int analogPin3 = 10;
int analogPin4 = 13;
int analogPin5 = 14;
int analogPin6 = 15;
int forceRead1 = 0;
int forceRead2 = 0;
int forceRead3 = 0;
int forceRead4 = 0;
int forceRead5 = 0;
int forceRead6 = 0;


/* select I2C channel using TCA9548A multiplexer */
void tcaselect(uint8_t channel)
{
//  Serial.print("I2C Channel: ");  Serial.println(channel);
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
}

void setup(void)
{
  Serial.begin(115200);
//  Serial.println(""); Serial.println("Orientation Sensor Test"); Serial.println("");

//  This is the vicon triggering stuff
//  pinMode(52,OUTPUT);
//  pinMode(52,HIGH);
//
pinMode(52,INPUT_PULLUP);
pinMode(50,OUTPUT);
digitalWrite(50,HIGH);


  Wire.begin();
//  This is how many IMUs you have connected to the system
  uint8_t IMUCount = 7;

//  Added one because we are skipping 1 port.
  uint8_t numImu = IMUCount+1;
  uint8_t ch;

/*  Sets up communications, if one is not working will return which one needs to be fixed.
 If broken, fiddle with connector cables as they are iffy at times.*/
  IMUCount = 0;
  for (ch=0; ch<numImu; ch++)  // multiple I2C devices
  {
    if (ch !=1){
    tcaselect(ch);
    /* Initialise the sensor */
    IMUCount = IMUCount+1;
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      Serial.println("");
      Serial.print(IMUCount);
      while(1);
    }
  }
  }
//  Displays the details of the sensors that you have set up
//  delay(1000);
//  for (ch=0; ch<numImu; ch++)  // multiple I2C devices
//  if (ch !=1){
//  {
//    tcaselect(ch);
//    /* Display some basic information on this sensor */
////    displaySensorDetails();
//  }
//}
}

void loop(){

//  Writes the pin low to create a falling edge for the Vicon System
//  digitalWrite(52, LOW);
//  delay(100);
//  if (digitalRead(52)==LOW){
if (digitalRead(52)==0){
  digitalWrite(50,LOW);
}

if (digitalRead(50)==0){
  delay(100);
  forceRead1 = analogRead(analogPin1);
  forceRead2 = analogRead(analogPin2);
  forceRead3 = analogRead(analogPin3);
  forceRead4 = analogRead(analogPin4);
  forceRead5 = analogRead(analogPin5);
  forceRead6 = analogRead(analogPin6);

//This outputs the IMU readings to Serial at baud rate of 115200
  /* IMU1 */
  tcaselect(0);
  sensors_event_t event1;
  bno.getEvent(&event1);
//  Serial.print("\t ");
  Serial.print(event1.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event1.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event1.orientation.z, 4);
  Serial.print(", ");

  /* IMU2 */
  tcaselect(2);
  sensors_event_t event2;
  bno.getEvent(&event2);
  Serial.print("\t ");
  Serial.print(event2.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event2.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event2.orientation.z, 4);
  Serial.print(", ");
//  Serial.println("");

    /* IMU3 */
  tcaselect(3);
  sensors_event_t event3;
  bno.getEvent(&event3);
  Serial.print("\t ");
  Serial.print(event3.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event3.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event3.orientation.z, 4);
  Serial.print(", ");
//  Serial.println("");

////
////
//////  /* IMU4 */
  tcaselect(4);
  sensors_event_t event4;
  bno.getEvent(&event4);
  Serial.print("\t ");
  Serial.print(event4.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event4.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event4.orientation.z, 4);
  Serial.print(", ");
//  Serial.println("");

//
////  /* IMU5 */
  tcaselect(5);
  sensors_event_t event5;
  bno.getEvent(&event5);
  Serial.print("\t ");
  Serial.print(event5.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event5.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event5.orientation.z, 4);
  Serial.print(", ");
//  Serial.println("");

////
////  /* IMU6 */
    tcaselect(6);
  sensors_event_t event6;
  bno.getEvent(&event6);
  Serial.print("\t ");
  Serial.print(event6.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event6.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event6.orientation.z, 4);
  Serial.print(", ");
//  Serial.println("");

/* This IMU is still broken and being fixed, will have it up and working when I get back */
////  /* IMU7 */
    tcaselect(7);
  sensors_event_t event7;
  bno.getEvent(&event7);
  Serial.print("\t ");
  Serial.print(event7.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event7.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event7.orientation.z, 4);
  Serial.print(",");
//  Serial.println("");
//

//  Print out the Hall readings.
// Add it to the 1x21 vector output in same CSV form
  Serial.print(forceRead1);
  Serial.print('\t');
  Serial.print(forceRead2);
  Serial.print(', ');
  Serial.print(forceRead3);
  Serial.print(', ');
  Serial.print(forceRead4);
  Serial.print(', ');
  Serial.print(forceRead5);
  Serial.print(', ');
  Serial.print(forceRead6);
  Serial.print("");
  }

/* This is extra stuff for quaternian calculations! */
////    imu::Quaternion q = bno.getQuat();
////    float temp = q.x();  q.x() = q.y();  q.y() = temp;  q.z() = -q.z();  // fly.c convention
////    q.normalize();
////    #if 0  // use broken BNO055 euler
////      /* The processing sketch expects data as roll, pitch, heading */
////      Serial.print(F("Orientation: "));
////      Serial.print( event.orientation.x);  // heading
////      Serial.print(F(" "));
////      Serial.print(-event.orientation.y);  // roll
////      Serial.print(F(" "));
////      Serial.print(-event.orientation.z);  // pitch
////      Serial.println(F(""));
////    #else
////      /* Convert quaternion to Euler, because BNO055 Euler data is broken */
////      imu::Vector<3> euler;
////      /* Adafruit's confusing x,y,z names are actually axis Z,Y,X rotations heading,roll,pitch */
////      euler.x() = 180/M_PI * atan2(q.w()*q.z() + q.x()*q.y(), 0.5 - q.y()*q.y() - q.z()*q.z());  // heading
////      euler.y() = 180/M_PI * atan2(q.w()*q.x() + q.y()*q.z(), 0.5 - q.x()*q.x() - q.y()*q.y());  // roll
////      euler.z() = 180/M_PI * asin(2 * (q.w()*q.y() - q.x()*q.z()));  // pitch
////      Serial.print(F("Orientation: "));
////      Serial.print(euler.x());  // heading, nose-right is positive, z-axis points up
////      Serial.print(F(" "));
////      Serial.print(euler.y());  // roll, rightwing-up is positive, y-axis points forward
////      Serial.print(F(" "));
////      Serial.print(euler.z());  // pitch, nose-down is positive, x-axis points right
////      Serial.println(F(""));
////    #endif
////
////    /* send quaternion */
////    Serial.print(F("Quaternion: "));
////    Serial.print(q.w(), 4);
////    Serial.print(F(" "));
////    Serial.print(q.x(), 4);
////    Serial.print(F(" "));
////    Serial.print(q.y(), 4);
////    Serial.print(F(" "));
////    Serial.print(q.z(), 4);
//    Serial.println(F(""));
//
//    /* Also send calibration data for each sensor. */
//    uint8_t sys, gyro, accel, mag = 0;
//    bno.getCalibration(&sys, &gyro, &accel, &mag);
//    Serial.print(F("Calibration: "));
//    Serial.print(sys, DEC);
//    Serial.print(F(" "));
//    Serial.print(gyro, DEC);
//    Serial.print(F(" "));
//    Serial.print(accel, DEC);
//    Serial.print(F(" "));
//    Serial.println(mag, DEC);
//  }
}
