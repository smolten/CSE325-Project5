#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"


Adafruit_GPS GPS(&Serial3);                   // define GPS object connected to Serial 3
DFR_Key keypad;  
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

#define GPSECHO  false

// Global variables that are changed across functions
int STEER_ANGLE = 90;       // servo initial angle (range is 0:180)
float HEADING = 0;  // heading
boolean usingInterrupt = false;
int carSpeedPin = 2;      // pin for DC motor (PWM for motor driver)
float errorHeadingRef = 0;        // error
long int lat = 0;  // GPS latitude in degree decimal multiplied by 100000
long int lon= 0;  // GPS latitude in degree decimal multiplied by 100000
long int latDestination = 0;//33.423933 * 100000;     // reference destination
long int lonDestination = 0;//-111.939585 * 100000;   // reference destination
long int diff_lat = 0;
long int diff_lon = 0;
float Bearing = 0;                // bearing angle to destination
int off_angle = 0;
int localkey = 0;                 // variable for keypad


//Tempe Boundaries
int LAT_MIN = 3341510;
int LAT_MAX = 3342310;

int LON_MIN = -11193860;
int LON_MAX = -11192670;


void setup() {
  myservo.attach(48);     // servo is connected to pin 48
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)
  Serial.begin(9600);     // serial for monitoring
  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) { //if you want to calibrate using another mode, set it here. OPERATION_MODE_COMPASS for a precise tilt compensated compass (Section 3.3.2 / 3.3.3)
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  ///Setting the reference (Lat and Lon)///
  localkey = 0;
  while (localkey != 1) {    // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to save dest.");
    delay(100);               // delay to make display visible
  }

  //while( lon == 0 || lat == 0) 
  {
    ReadGPS();
    delay(1000);  //wait for GPS fix before setting destination
    lcd.clear();
    lcd.print("Waiting for GPS");
    lcd.setCursor(0, 1);
    lcd.print("lat:");lcd.print(lat,5);
    lcd.print("lon:");lcd.print(lon,5);
  Serial.println(lat);
  Serial.println(lon);
  Serial.println();
  }
  
  latDestination = lat;     // saving the destiantion point
  lonDestination = lon;     // saving the destiantion point
  localkey = 0;
  while (localkey != 1) {   // wait for select button2
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);             // delay to make display visible
  }


  byte c_data[22] = {0, 0, 0, 0, 0, 0, 211, 3, 117, 4, 55, 5, 255, 255, 255, 255, 255, 255, 232, 3, 21, 3};
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);
  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);  // enable timer compare interrupt
  interrupts();


  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate // it's more stable than 10Hz
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
}

SIGNAL(TIMER0_COMPA_vect) { // leave this function unchanged//
  char c = GPS.read();    // this function is for reading chars from GPS module
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) { // This function will be called every 1 second
  sei();        //   set interrupt flag // don't change this
  TCNT4  = 336; //   re-initialize timer4's value
  ReadGPS();    //   read GPS data
  
}

ISR(TIMER1_OVF_vect) {        // This function will be called every 0.1 second
  sei();                  // set interrupt flag // don't change this
  TCNT1  = 59016;         // reinitialize the timer1's value
  ReadHeading();          // read heading
  CalculateBearing();     // calc bearing
  CalculateSteering();    // calc steering
  CalculateDistance();    // calc distance
  Actuate();              // Actuate
}


void ReadGPS() {
  // read from GPS module and update the current position
      if (GPS.newNMEAreceived())
          GPS.parse(GPS.lastNMEA()); //Parse GPS Sentences
          
      if (GPS.fix) 
     { //if at least five fixed satellites are found
          //Update latitude and longitude values
           float raw_lat = GPS.latitude; 
           long int long_lat = (long)(raw_lat * 100.0);
           float lat_minutes = (float)(long_lat % 10000) / 100; // get the lower 4 digits of the raw data (which is the minutes)
           long int lat_degrees = (long)(long_lat / 10000);
           long int tmp_lat = (lat_degrees + (lat_minutes / 60)) * 100000;
           //if (tmp_lat != 0)
           //if (tmp_lat > LAT_MIN && tmp_lat < LAT_MAX) 
           { lat = tmp_lat; }

           float raw_lon = GPS.longitude;
           long int long_lon = (long)(raw_lon * 100.0);
           float lon_minutes = (float)(long_lon % 10000) / 100; // get the lower 4 digits of the raw data (which is the minutes)
           long int lon_degrees = (long)(long_lon / 10000);
           long int tmp_lon = -(lon_degrees + (lon_minutes / 60)) * 100000;
           //if (tmp_lon != 0)
           //if (tmp_lon > LON_MIN && tmp_lon < LON_MAX) 
           { lon = tmp_lon; }

           if (latDestination == 0) {latDestination = lat;}
           if (lonDestination == 0) {lonDestination = lon;}

           
          Serial.println("Have fix");
    Serial.print("tmplat:");Serial.println(tmp_lat); //lat and lon stay 0
    Serial.print("tmplon:");Serial.println(tmp_lon);
    Serial.print("lat:");Serial.println(lat); //lat and lon stay 0
    Serial.print("lon:");Serial.println(lon);
      } 

}

void ReadHeading() { // Output: HEADING
  // read Heading angle
  imu::Vector<3> eulVect = bno.getVector(Adafruit_BNO055::VECTOR_EULER);      // Euler Vector
  HEADING = (eulVect.x() + 20);  
}

void CalculateBearing() {
  // calculate bearing angle based on current and destination locations (GPS coordinates)
  diff_lat = latDestination - lat;
  diff_lon = lonDestination - lon;
  Bearing = atan2(diff_lat, diff_lon);
}

void CalculateSteering() { // Input: HEADING // Output: STEERANGLE// Calculate the steering angle according to the referece heading and actual heading
  int dispAngle = Bearing - HEADING;          // angle of displacement from heading and reference
  while(dispAngle<0) 
    dispAngle += 360;
  while(dispAngle >360)
    dispAngle -= 360;  

  off_angle = dispAngle;

  //left:0 right:180 neutral: 90
  //dispAngle 0->170  ->   0 angle = Left(0) servo, 180 angle = Neutral(90)
  //dispAngle 190->360  -> 180 angle = Neutral(90),  360 angle = Right(180)
  STEER_ANGLE = dispAngle / 2;
}

void CalculateDistance() {
  // calculate distance to destination based on current and destination coordinates
  long distancebig = sqrt(diff_lat*diff_lat + diff_lon*diff_lon);
  
}

void Actuate() {
  // set car's direction and speed
  myservo.write(STEER_ANGLE);
}

void printHeadingOnLCD() {

}

void printLocationOnLCD() {
  lcd.print("lat diff");
  lcd.print(diff_lat);
  lcd.setCursor(0, 1);    // new line
  lcd.print("lon diff");
  lcd.print(diff_lon);

  
  //Serial.println(lat);
  //Serial.println(lon);
  //Serial.println();
}

void printDistanceOnLCD() {

}

void loop() {
  lcd.clear();    // clear the LCD
  // You can print anything on the LCD to debug your program!!!
  //printHeadingOnLCD();
  printLocationOnLCD();
  delay(100);
}
