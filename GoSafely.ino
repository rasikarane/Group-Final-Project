#include "I2Cdev.h"

#include "Ultrasonic.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define RED_FW 12
#define RED_BW 9
#define YEL_LEFT 10
#define YEL_RIGHT 11
#define GREEN 8

float p_rest = 2;
float r_rest = -0.5;

int cnt = 0;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3];           

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
//end gyro-------------------------------------------------------------------

//gps--------------------------------------------------------------------------------------
String monthDB[] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
char input = ' ';
float lat,lon = 0;

//Uncomment the lat and lon you'd like to route to
//ArmStrong Student Center
//double targLat = 39.507424;
//double targLon = -84.733231;
//Benton
//double targLat = 39.510545;
//double targLon = -84.733510;
//King Library
//double targLat = 39.508805;
//double targLon = -84.737709;
//Skyline
//double targLat = 39.510399;
//double targLon = -84.742204;
//Walmart
//double targLat = 39.528480;
//double targLon = -84.767998;
//Goggin
//double targLat = 39.504319;
//double targLon = -84.736587;
//Western Dining Hall
//double targLat = 39.504906;
//double targLon = -84.727877;
//Millet
//double targLat = 39.516615;
//double targLon = -84.734566;
//New york <<<350 5th Ave New York NY 10118
//double targLat = 40.7481628;
//double targLon = -73.9849957;
//Chicago <<<Main Street, West Chicago, DuPage County, Illinois, 60185
double targLat = 41.8820006;
double targLon = -88.1995096;
//Texas <<<61 Riesner St, Houston, TX 77002
//double targLat = 29.7646147;
//double targLon = -95.3711651;
//Florida <<<2201 NE 163rd St, North Miami Beach, FL 33160
//double targLat = 25.9264193;
//double targLon = -80.1555557;

float compass,knots = 0;
double targcomp = 0;
double compDif = 0;
//end gps----------------------------------------------------------

//ultra------------------------------------------------------------
Ultrasonic ultrasonic (6,7);

int vib1 = 4;
int vib2 = 5;

long microsec = 0;
float distanceCM = 0;

int notes[] = {300, 600};
//ultra--------------------------------------------------------------

//gyro-------------------------------------------------------------
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//end gyro----------------------------------------------------------


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================ 

void setup() {

//gps-------------------------------
    Serial.begin(9600);
    Serial.println("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
    Serial.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
    Serial.println("$PMTK251,9600*17");
    Serial.println("$PMTK220,200*2C");
    Serial.println("$PMTK300,200,0,0,0,0*2F");
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(3, OUTPUT);
//end gps-----------------------------------------

//ultra-------------------------------------------------
    pinMode(vib1, OUTPUT);
    pinMode(vib2, OUTPUT);
//end ultra------------------------------------------------

//gyro-----------------------------------------------------    
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1688);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);

        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}
//end gyro---------------------------------------------------------


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
//gps ------------------------------------------------------------
  String thing,gmt,latstr,ns,lonstr,ew,knotstr,compstr,day,month,year = "";
  boolean fix = false;
    if(Serial.read() == '$'){
      thing = Serial.readStringUntil('$');
    }
    if(thing[3] == 'M') {
      char current[thing.length()];
      thing.toCharArray(current,thing.length());
      int i = 0;
      
      //skip the letters
      while(current[i] != ','){
        i++;
      }
      i++;

      //get Time
      gmt = current[i];
      i++;
      gmt += current[i];
      gmt += ':';
      i++;
      gmt += current[i];
      i++;
      gmt += current[i];
      while(current[i] != ','){
       i++;
      }
      i++;

      //Check Fixed or not
      if(current[i] == 'A'){
        fix = true;
      }
      i += 2;

      //get Latitude
      while(current[i] != ','){
        latstr += current[i];
        i++;
      }
      i++;

      //get North/South
      if(current[i] == 'N'){
        ns = "North";
      } else if(current[i] == 'S'){
        ns = "South";
      }
      i += 2;

      //get Longitude
      while(current[i] != ','){
        lonstr += current[i];
        i++;
      }
      i++;

      //get East/West
      if(current[i] == 'E'){
        ew = "East";
      } else if(current[i] == 'W'){
        ew = "West";
      }
      i += 2;

      //get knots
      while(current[i] != ','){
        knotstr += current[i];
        i++;
      }
      i++;

      //get compass direction
      while(current[i] != ','){
        compstr += current[i];
        i++;
      }
      i++;

      day += current[i];
      i++;
      day += current[i];
      i++;

      month += current[i];
      i++;
      month += current[i];
      i++;

      month = monthDB[month.toInt() - 1];

      year += current[i];
      i++;
      year += current[i];
      i++;
      
      //Testing Converting to Decimal Degrees
      
      //latstr = "3857.5634";
      //ns = "North";
      //lonstr = "9515.9289";
      //ew = "West";
      

      lat = latstr.toInt() / 100;
      lat += (latstr.toFloat() - lat*100) / 60;
      if(ns == "South"){
        lat *= -1;
      }

      lon = lonstr.toInt() / 100;
      lon += (lonstr.toFloat() - lon*100) / 60;
      if(ew == "West"){
        lon *= -1;
      }

      //Forcing Data to test without reads
      lat = 39.5060848;
      lon = -84.7278971;

      compass = compstr.toFloat();
      knots = knotstr.toFloat();

      /* Inverted Angles, keeping incase it becomes useful
      if(targLat > lat && targLon < lon){
        targcomp = atan((lon-targLon)/(targLat-lat))*(180/3.14);
      } else if(targLat < lat && targLon < lon){
        targcomp = (atan((lat-targLat)/(lon-targLon))*(180/3.14)) + 90;
      } else if(targLat < lat && targLon > lon){
        targcomp = (atan((targLon-lon)/(lat-targLat))*(180/3.14))  + 180;
      } else if(targLat > lat && targLon > lon){
        targcomp = (atan((targLat-lat)/(targLon-lon))*(180/3.14)) + 270;
      } else {
        targcomp = 0;
      }
      */

      //Determines the angle at which the user needs turn
      if(targLat > lat && targLon < lon){
        targcomp = atan((targLat-lat)/(lon-targLon))*(180/3.14) + 270;
      } else if(targLat < lat && targLon < lon){
        targcomp = (atan((lon-targLon)/(lat-targLat))*(180/3.14)) + 180;
      } else if(targLat < lat && targLon > lon){
        targcomp = (atan((lat-targLat)/(targLon-lon))*(180/3.14))  + 90;
      } else if(targLat > lat && targLon > lon){
        targcomp = (atan((targLon-lon)/(targLat-lat))*(180/3.14));
      } else {
        targcomp = 0;
      }

      //Forcing Data
      compass = 160;

      //Accounts for users current direction to prepare to calculate turn
      compDif = targcomp - compass;
      if(compDif < 0){
        compDif += 360;
      }

      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
      digitalWrite(12,LOW);
      digitalWrite(13,LOW);
      digitalWrite(3, LOW);

      //Forcing Data
      fix = true;

      //Tells user to turn according to current path
      if(fix){
        if(compDif <= 5 || compDif >= 355){
          //Forward
          digitalWrite(10,HIGH);
        } else if(compDif > 5 && compDif <= 90){
          //Right
          digitalWrite(11,HIGH);
        } else if(compDif > 90 && compDif <= 270){
          //Behind
          digitalWrite(12,HIGH);
        } else if(compDif > 270 && compDif <= 355){
          //Left
          digitalWrite(13,HIGH);
        } else {
          digitalWrite(3,HIGH);
        }
      }
    }
//end gps----------------------------------------------------------

//ultra------------------------------------------------------------
    microsec = ultrasonic.timing(); 
    distanceCM = ultrasonic.convert(microsec, Ultrasonic::CM); 

    ledDistance();

    delay(100);
//end ultra-------------------------------------------------------

//gyro------------------------------------------------------------    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    
    while (!mpuInterrupt && fifoCount < packetSize) {
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            float pitch = (ypr[1] * 180/M_PI);
            float roll = (ypr[2] * 180/M_PI);
           
            if(cnt < 350){
              cnt++; 
              noTone(3);
            }else{
              if((abs(pitch - p_rest) <= 5) && (abs(roll - r_rest) <= 5)){
                //digitalWrite(GREEN, HIGH);
                noTone(3);
              }else{
                if((pitch - p_rest) > 5){
                  //digitalWrite(RED_BW, HIGH);
                  tone(3,notes[0]);
                }else if((pitch - p_rest) < -5){
                  //digitalWrite(RED_FW, HIGH);
                  tone(3,notes[0]);
                }
                if((roll - r_rest) > 5){
                  //digitalWrite(YEL_RIGHT, HIGH);
                  tone(3,notes[0]);
                }else if((roll - r_rest) < -5){
                  //digitalWrite(YEL_LEFT, HIGH);
                  tone(3,notes[0]);
                }
              }
            }
        #endif
    }
}
//end gyro---------------------------------------------------------

//ultra------------------------------------------------------------
void ledDistance() {

  digitalWrite(vib1, LOW);
  digitalWrite(vib2, LOW);

  if (distanceCM > 70) {
  }

  if (distanceCM <=70 and distanceCM >= 30) {
    digitalWrite(vib1, HIGH);
  }

  if (distanceCM < 30) {
    digitalWrite(vib1, HIGH);
    digitalWrite(vib2, HIGH);
  }
}
//end ultra--------------------------------------------------------
