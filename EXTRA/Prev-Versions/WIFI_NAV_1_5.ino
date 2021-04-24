// by Jonathan and Max 
//#ifdef ESP32
#include <WiFi.h>
#include <HTTPClient.h>
#include <FirebaseESP32.h>
//#include <FirebaseJson.h>

//Txt files
#include <SPIFFS.h>
#include "FS.h"

#include <Arduino.h>

#include <Wire.h>         // I2C 
//#include <LIS3MDL.h>      // Compass
//#include <LSM6.h>         // Gyro and Accel 

//LSM6 imu;                 // Gyro and accel instance 

// Set web server port number to 80 - SoftAP
WiFiServer server(80);

// ESP32 wifi network connection data
#define ssidEspServer "ESP32Server"
#define PassEspServer "87654321"

// Connection data to the user's local wifi network
#define ssidWiFiLocalSet "iPhone"
#define passwordSenhaWiFiLocal "jonathan"

//Data for connection to the "Firebase" database
#define FIREBASE_HOST "https://lawnmower-android-66b33.firebaseio.com"
#define FIREBASE_AUTH "4YOEWZHxqEOpZokJi5bhdcMpuIy15Qwgh45MKTO2"


//Define FirebaseESP8266 data object for data sending and receiving
FirebaseData fbdo;

//Variables
String receiveCar; // Car Received Data
String sendReceive; // Selection of active Tasks
String firebaseReceive; // Data Received from Firebase
String strJsonFirebase; // Data Json Received from Firebase
String strJsonClient = "{\"battery\":\" \",\"start_status\":\" \",\"latitude\":\" \",\"longitude\":\" \",}"; // Data Json Received from client

String clientInput; // Car Received Data
String latitude;  // Car Received Data
String longitude; // Car Received Data
String battery; // Received from the car

String start_status; // Received from Firebase
String _0_latitude;  // Received from Firebase
String _0_longitude; // Received from Firebase
String _1_latitude;  // Received from Firebase
String _1_longitude; // Received from Firebase
String _2_latitude;  // Received from Firebase
String _2_longitude; // Received from Firebase
String _3_latitude;  // Received from Firebase
String _3_longitude; // Received from Firebase


// Semaphore allows access to variables by different tasks at different times
SemaphoreHandle_t myMutex;

// Method that allows to use "Serial.print ()" in different tasks
String str_global = "";
void printGlobal(String str) {
  xSemaphoreTake(myMutex, portMAX_DELAY);
  str_global = str;
  Serial.println(str_global);
  xSemaphoreGive(myMutex);
}
// ------------------------------------------- NAVIGATION VARIABLES//------------------------------------------//

double lat1;
double lat2;
double lon1;
double lon2;
double latR1;
double latR2;
double lonR1;
double lonR2;
double dlon;
double dlat;
double a;
double e;
double d;
double R = 6371.00;
double toDegrees = 57.295779;
char sb[10];

// the number of the PWM pin

const uint8_t PinL_F  = 12;  // left forward PWM   //******************************************************************************************************//

const uint8_t PinL_R  = 13;  // left reverse pwm  //******************************************************************************************************//

const uint8_t PinR_F  = 27;  // right forward pwm 

const uint8_t PinR_R  = 26;  // right reverse pwm  

const uint8_t PinL_1  = 2;  // left motor forward enable  

const uint8_t PinL_2  = 15;  // Left motor reverse enable  

const uint8_t PinR_1  = 4;  // Right Motor Forward enbable 

const uint8_t PinR_2  = 18;  // Right Motor Reverse Enable  

const uint8_t Pin_RSE = 19;   // Right SE indicator channel GPIO 26 m.lesser 4/3/21

const uint8_t Pin_LSE = 5;   // Left SE indicator channel GIPO 27 m.lesser 4/3/21

const uint8_t Pin_LA = 32;        // left ch a se pin 
 
const uint8_t Pin_RA = 33;     // right ch A pin   

const uint8_t O_ALERT = 35;     // ostacle alert pin from pic    

const uint8_t Blade = 0; //PINNUM;  //******************************************************************************************************// 
//******************************************************************************************************//                              
// Global Vars for se indicator counts  
int L_SE_CNT_GLOB = 0; 
int R_SE_CNT_GLOB = 0; 
int L_A_GLOB = 0; 
int R_A_GLOB = 0; 

 
//******************************************************************************************************// 
// setting PWM properties

const uint32_t freq = 5000;  

const uint8_t L_Channel = 1; // note channel 0 and 1 share frequencies

const uint8_t R_Channel = 2; // note channel 2 and 3 share frequencies

const uint8_t resolution = 8;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//******************************************************************************************************// 
// setting up constants 



const uint8_t CPR = 48;         // counts per revoltuion on ch A                                                // validated

//**************************************************// 
// speed settings for different opperations 

const uint8_t STOP = 7;        // base pwm setting for stopping 

// settings on concrete 
// start at 50
// 10 -> 30 
// 30 -> 60 
const uint8_t TURN_STOP = 60;   // PWM setting to lock inside wheel. STOP speed insuficient for turn stop 

// settings on concrete 
// 170 -> 150
// 170 -> 210
// 170 -> 85
// 100 -> 150
// 130 -> 100
const uint8_t TURN = 100;       // PWM setting for turning. to be applied to only one motor while the other is stopped. 

const uint8_t SLOW = 64;        // slow speed setting 

// 100 -> 50
const uint8_t MED = 70;        // medium speed setting 

const uint8_t FAST = 225;       // fast speed setting 




 // each shaft roation corresponds to ~ 3/4 of a wheel rotation, where the wheel diameter is ~ 7.75 inches
 // Wheel circumference is ~ 63.5 cm, meaining each shaft rotation is ~ 47.625 cm => 48 cm ? 
 const double Rot_dist = 21.43125; // 47.625; // liner distnace travled in one shaft rotation                                // check this number                                          

 const double trackWidth = 15;    // track width of mower in cm ( guessed at 15 cm for now) 

 // 90 deg turn rotation distance for left wheel 

 const double TurnDist = 79.8;    // rotation of left wheel needed to compleate 90 deg turn                    // check this too 
 //******************************************************************************************************// 

/*********************************************************************************/ 
  // Nav variables 
                                                       //******************************************************************************************************// 
                                                     //******************************OI LOOK HERE YA BAT ****************************************************// 
                                                     //******************************************************************************************************// 

  bool correction = false;         // set to true to correct wheel speed to go straight. If things fuck up, set to false 

  uint8_t med_L = MED;      // holds adjusted speed for left wheel in go straight function 

  uint8_t med_R = MED;      // holds adjusted speed for right wheel in go straight function 

  int ind = 1;              // incrementally boosts reversing power in stop function 

  bool stopped = false;     // tracks if we have stopped in stopping function 

  int LegNum = 0;     // tracks what leg we are on 

  int LoopNum = 0;    // Tracks which loop around the yard we're on

  int L_last = 0;   // hold last encode value on channel A, so check if we've actully stopped 

  int R_last = 0;   // same as above, but for right. both used in stop function 

/*********************************************************************************/ 
// outside box width 
// get legs from W-MCU 

double Leg0 = 3;      // Leg 0 distance in M 

double Leg1 = 3;      // Leg 1 distance

double Leg2 = 3;      // Leg 2 distnace 

double Leg3 = 3;      // Leg 3 distance 

double LegAr[4] = {Leg0, Leg1, Leg2, Leg3}; 



const uint8_t LoopsNeeded = floor ((LegAr[0]*100)/Rot_dist); // (Leg0*100)/(2*trackWidth));    // total number of loops needed, based on leg0 in the group

float Gyro_Cal = 0.0;     // gyro calibration dummy var 

bool OBSTACLE = false;    // ostacle detected

bool evading = false;     // evading obstacle  


//******************************************************************************************************// 
//******************************************************************************************************// 
// ISRS 
//******************************************************************************************************// 
//******************************************************************************************************// 

// ISR m.lesser 4/3/21
//////////////////////////////////////////////
// LEFT SE NOT TRIGGERING 
/////////////////////////////////////////////
// ISR to incriment left SE cnt 

void IRAM_ATTR L_SE_ISR() {     
  L_SE_CNT_GLOB ++; 
  
  
}

// ISR to incriment right SE cnt 
void IRAM_ATTR R_SE_ISR() {
  R_SE_CNT_GLOB++;
 
}

void IRAM_ATTR R_A_ISR() {
  R_A_GLOB++;
  
}


void IRAM_ATTR L_A_ISR() {
  L_A_GLOB++;
}

/* ISR causes unexcpeted opperation 
void IRAM_ATTR ODET()
{ 
  // if OD alert pin sets high, set the OBSTACLE bool to high 
  OBSTACLE = true; 
  //Serial.println("Found it"); 
  //delay(2000); 
}
*/ 

// ------------------------------------------- NAVIGATION FUNCTIONS//------------------------------------------//
/// sets left motor to go forward 
void Left_F(int Duty_Cyc)
{
  ledcWrite(1,Duty_Cyc); 
  ledcWrite(2,0); //(PinL_R, LOW); 
  //Serial.println("Left motor Forward"); 
}
//******************************************************************************************************// 

// sets left motor to go reverse 
void Left_R(int Duty_Cyc)
{
  ledcWrite(2,Duty_Cyc);
  ledcWrite(1,0); //(PinL_F, LOW);
  //Serial.println("Left motor Reverse");  
  
   
}
// coasts left motor ( i hope) Does not work as expected. prob need to set digi pins low 
void Left_C()
{
  ledcWrite(1,255); //(PinL_F, LOW);
  ledcWrite(2,255); //(PinL_F, LOW);
  //digitalWrite(PinL_F, HIGH);
  //digitalWrite(PinL_R, HIGH); 
  //Serial.println("Left motor coast");  
}
// notice: motor oppose each other, so forward on right is reverse on left. 
// functions are therefor backwards 
// right motor forward
//******************************************************************************************************// 
 
void Right_R(int Duty_Cyc)
{
  ledcWrite(4,Duty_Cyc);
  ledcWrite(3,0); // digitalWrite(PinR_R, LOW); 
  //Serial.println("right motor reverse"); 
   
}
//******************************************************************************************************// 

// right motor reversen 
void Right_F(int Duty_Cyc)
{
  ledcWrite(3,Duty_Cyc);
  ledcWrite(4,0); 
  //digitalWrite(PinR_F, LOW);
  //Serial.println("right motor forward"); 
}
//******************************************************************************************************// 
// coasts right motor ( i hope) // see left_c 
void Right_C(void)
{
  ledcWrite(3,255);
  ledcWrite(4,255);
  //digitalWrite(PinR_F, HIGH);
  //digitalWrite(PinR_R, HIGH);
  //Serial.println("right motor coast"); 
  
}
//******************************************************************************************************// 
// Stop Function 
void Stop()
{
  //Serial.println("Stopping"); 
  // dosen't fall out of loop. attempt to use I channel instead 
  attachInterrupt(Pin_LA, L_A_ISR, RISING);

  attachInterrupt(Pin_RA, R_A_ISR, RISING);

  R_A_GLOB = 0;   // resetting ch a encoder counts  

  L_A_GLOB = 0; 

  //R_SE_CNT_GLOB = 0; 

  //L_SE_CNT_GLOB = 0; 

  L_last = 0;   // hold last encode value on channel A, so check if we've actully stopped 

  R_last = 0; 

  stopped = false;      // resetting bool 
  
  // stops mower 
  // breaking by applying low power reverse 
  // breaks Left Motor 
  Right_R(STOP); 
  
  Left_R(STOP); 

   

  ind = 1;              // incrementally boosts reversing power 

  while((!stopped) & (ind <= 15)) // 
  { 
    L_last = L_SE_CNT_GLOB;    // last SE- ch A count becomes current CH- A count for both wheels 

    R_last = R_SE_CNT_GLOB; 

    ind++;                // seems to not fall out of loop. adding this to make sure it does 

    delay(100);           // delay by 0.1 seconds. assumption is that if we have not moved at least 1/48th of a rotation in 0.1 seconds we are stopped 

    if ((L_last == L_A_GLOB) & (R_last == R_A_GLOB))   // gave error margin of 3/48                                                   // possibly include error bound here? 
    { // if both left and right wheel haven't moved more than 1/48th of a rotation in the last 0.1 seconds
      // mower is stopped 
      stopped = true; 
     
    }
    
    else if (correction)        // if course correction is active. Bool set at top 
    {
      if ((L_last == L_A_GLOB) && (R_last != R_A_GLOB)) 
      { 
        // left wheel stopped but right has not 
        ledcWrite(2,STOP + (3*ind));  // progressivly increase reverse thrust on right wheel 
        ind ++; 
        
      }
      else if ((L_last != L_A_GLOB) & (R_last == R_A_GLOB)) 
      { 
        // if right wheel has stopped and left has not 
        ledcWrite(1,STOP +(3*ind));   // progresivly increase reverse thrust on left wheel
        ind ++; 
      }
    }
    
  }

  // if we have stopped, turn both motors off and detach interrupts 

  detachInterrupt(Pin_LA);

  detachInterrupt(Pin_RA);
  
  //Right_C(); 

  //Left_C(); 

  
}
// Go straight function
//******************************************************************************************************// 
// Go straight function

void GoStraight(int Rotations)
{
  // sets output to go straight, for dist 
  // straight for left 
  //Serial.printf("Going Straight for %f", dist); 
  
  if (correction) 
  {
    // setting up for course correction 
    // to adjust left and right wheel speed to keep the same number of rotation on both wheels
    attachInterrupt(Pin_LA, L_A_ISR, RISING);

    attachInterrupt(Pin_RA, R_A_ISR, RISING);

    L_A_GLOB = 0;  

    R_A_GLOB = 0; 

    // we have a choice here. we can either reset left and right wheel to medium speed each itteration, as we do here 
    // OR we can not do that, under the assuption that the correction function find the correct speed for both motors, 
    // and that they stay more or less true after the first correction cycle 
    // may depend on variability of terrain over the current course. 

    med_L = MED;  

    med_R = MED; 
    
  }
  Left_F(MED); 

  Right_F(MED); 
  
  
   
  //Reverse for right motor

  

  // at full speed 
  //int Rotations = floor(dist/Rot_dist); // num of shaft rotations needed to travel dist;  

  R_SE_CNT_GLOB = 0;    // reset for accurate tracking 

  while ((R_SE_CNT_GLOB < Rotations))
  {
    
    if (correction) 
    { 
      // couse correction. slow one wheel if the difference in rotation is greater than 1/12th 
      // if correction is to aggresive / weak, can change either the delay we give the othwer wheel to catch up. 
      // or the speed decrese for the offencting wheel 
      if(abs(L_A_GLOB - R_A_GLOB) > 4)          // if we are off by more than 1/12th rotation between the 2 wheels
      { 
        if(L_A_GLOB > R_A_GLOB)                 // left wheel faster than right 
        { 
          med_L = med_L - 1;            // decreasing left wheel PWM by 1          
          ledcWrite(1,med_L);  
          delay(50);                    // giving right wheel time to catch up 
          
        }
        else                                    // right wheel faster than left 
        { 
          med_R = med_R - 1;            // decreasing right wheel PWM by 1           
          ledcWrite(2,med_R); 
          delay(50);                    // giving left wheel time to catch up        
          
        }
      }
    }
    
    //delay(10);    // used as NOP instruction here I feel like i should take this out, but I'm leaving it here cause that's what i last ran at 
    //Serial.printf("On rot num %d ", R_SE_CNT_GLOB); 
    //Serial.println("Going"); 
       //delay(10);    // used as NOP instruction here 
    //Serial.printf("On rot num %d ", R_SE_CNT_GLOB); 
    //Serial.println("Going"); 
    if ( digitalRead(O_ALERT == HIGH) ) // & (!evading)) // insert to only evade once 
    {
      Serial.println("Begin Obstacle Evasion Now"); 
      Clear_Obstacle(); 
    }
    
  

  // coasting both wheels after we've compleated the straigh leg 

  //Right_C(); 

  //Left_C(); 

  
  if(correction)
  {
    detachInterrupt(Pin_LA);

    detachInterrupt(Pin_RA);
  }

  }
}



//******************************************************************************************************// 
void Clear_Obstacle(void)
{ 
  // Obstacle avoidance setup 
  // First thing we need to do is stop dead 
  /*default to turn to right, so that we can keep the obstacle on out 
   * left, and in view of the left side sensors. 
   */
  bool TimeBased = true;      // shaft encoder malfunction prevents us from using rotation based evasion  
  Stop(); 
  evading = true; 
  OBSTACLE = false; // need to reset here to not fall into evasion repeatedly, i think                                                 // check here// 
  delay(1000);    // making sure we are actually stoped/ and or time to coast down 
  // saving context as we enter obstacle avoidance 
  int R_Context = R_SE_CNT_GLOB; 
  int L_Context = L_SE_CNT_GLOB; 
  // Attaching Interupts. 
  // Detached at end of this function
  attachInterrupt(Pin_LA, L_A_ISR, RISING);

  attachInterrupt(Pin_RA, R_A_ISR, RISING); 
 
  
  ////////////////////////////////////////////////////////////////////////////////
  // reading OD info from pic 
  // OD string should have following format" 
  // dd,s <distance,Side> where distanceis in CM, <99 cm, and side is either R,L,C 
  // ALSO:: SET ALERT PIN AND SEND STRING ONLY AT DISTANCE OF 50CM 
  // Baud Rate, Protocal ( need to verify with pic) TX and RX Pins (Need to assign) 

  if (!TimeBased)
  {   
    L_A_GLOB = 0; 
    R_A_GLOB = 0; 
    
      
    Serial.println("Turning out rigth"); 
        // Spin left wheel until we are no longer looking at the obstacle 
        // pins should be set just once in setup, as we will not be reversing
         
    //////////////////////////////////////////////////////////////////////////////// 
    // turining out 
    Left_F(SLOW); 
    Right_R(STOP);     
        // slow speed on left motor  
    bool cleared = false; 
    while (!cleared) //(O_ALERT == HIGH) | ( L_A_GLOB < 130)) // changed to absolute value floor((TurnDist/Rot_dist)*CPR)))
    {
      if (O_ALERT == LOW)
      {
        cleared = true; 
         
      }
      else if( L_A_GLOB < 130)    // 90 degs
      {
        cleared = true; 
        
      }
      // keep turning out until we hav cleared the obstacle, or turned by 90 degrees  
    }
  
  
    Stop(); 
    // Keeping track of how much we stepped out 
    //float SideStepRots = 0.0; 
    int LASideSteps = L_A_GLOB;     // rotations we have turend out by 
    // note, we use A channel here, but it does not matter, from how we're driving the wheels we know we're goind forward
    // and CH A & B both have 500CPR, just want an absoulte value for wheel rotations 
   
    //SideStepRots = (L_A_GLOB/CPR);     // L-CH-A CPR holds CPRS for both left and right A channel 
  
  
    // Distance we need to move at new angle to clear obstacle 
    // removed dist_cm 
    float ClearDist = 200; // 80 + 63.5 + 20;      // dist_cm is reported distance to obstacle, 63.5 (cm) is length from Array to end of mower, 20 is safety factor; 
    // num rotations associated with this distance 
    int ClearRots = ceil(ClearDist/Rot_dist);   // taking ceil here just in case 
  
    // clearing obstacle 
    ////////////////////////////////////////////////////////////////////////////////
    // going around 
    Serial.println("clearing"); 
    R_SE_CNT_GLOB = 0; 
    L_SE_CNT_GLOB = 0;
    GoStraight(ClearRots); 
  
  
    
    // clearing obstacle 
    ////////////////////////////////////////////////////////////////////////////////
    // coming back  
    cleared = false;  
    L_A_GLOB = 0; 
    R_A_GLOB = 0;
    Right_F(SLOW); 
    Left_R(7);  
    Serial.println("Straightening out to left");
        
    while(!cleared); // R_A_GLOB < CPR*SideStepRots)    // should be integer  
    {
      if (R_A_GLOB >= (2*LASideSteps))
      { 
        cleared = true; 
      }
    }
    Stop(); 
  
    // clearing obstacle 
    ////////////////////////////////////////////////////////////////////////////////
    // coming back  
  
    Serial.println("returning to original path"); 
    R_SE_CNT_GLOB = 0; 
    L_SE_CNT_GLOB = 0; 
    GoStraight(ClearRots); // retracing step from above 
  
    // clearing obstacle 
    ////////////////////////////////////////////////////////////////////////////////
    // straightening out 
    R_A_GLOB = 0; 
    L_A_GLOB = 0; 
    Serial.println("straightening back out to right"); 
    cleared = false; 
    Left_F(SLOW); // ledcWrite(1,SLOW);
    Right_R(STOP); 
    
    while (!cleared)//  L_A_GLOB < SideStepRots * CPR)
    { 
      if (L_A_GLOB >= LASideSteps); 
      { 
        cleared = true; 
      }
         
    }
    Stop(); 
  
    
    // we are now clear of the obstacle, and pointing back into the original direction 
  
    // Finnally we need to restore the shaft encoder contexts
     // NOW need to adjust linear distance 
       // Formula: 
       // (side_rots/(rots_for_90_deg) * (pi/2) : angle in rads that we sheered out -> phi 
       // clearance rots: distance we traveled at that angle (ie hypothenouse of our triangle) 
       // SideStep_Rots: distance we traveled parallel to out path (may be 0 for small obstacles) 
       // Liner distance along original path (Lin_Dist_ = 2*((cos(phi))*clerance_rots) + SideStep_Rots
       //                                      sheer out +sheer back in     + parallel dist 
  
    float phi = ( ( (LASideSteps /CPR) *Rot_dist)/ TurnDist) *(3.141/2);       // angle in rads that we turned out 
    int Lin_Dist = (2*cos(phi)*ClearRots); //+ ParaRots;                      // not taking into account wheel circumference, as it sould drop out as a factor 
    R_SE_CNT_GLOB = R_Context + Lin_Dist; 
    L_SE_CNT_GLOB = L_Context + Lin_Dist; 
  
    // detach interupts here 
    detachInterrupt(Pin_LA);
  
    detachInterrupt(Pin_RA);
   
    ///////////////////////////////////////////////////////////////////////////////////////////
    // NOTE 
    // L-SE-A only runs about 3CPR when it should be 500. have adjusted code to use 3 as CPR count. 
    // Do not know how reliable the 3 number is. 
    // May need to compleatly remove left SE from program 
    
    //OBSTACLE = false;     // we have cleared the obstacle, and can set it's bool to false  
  }
  else 
  { 
    digitalWrite(Blade, LOW); // turn off blade while evading 
    // stop 
    Right_R(10); 
    Left_R(10); 
    delay(5000); 
    // turing out 
    Left_F(TURN); 
    Right_R(STOP+15); 
    delay(1000);
    // going past  
    Left_F(SLOW); 
    Right_F(SLOW); 
    delay(900); 
    // turning back towards original path 
    Left_R(STOP); 
    Right_F(TURN); 
    delay(2000);     
    // coming back 
    Left_F(SLOW); 
    Right_F(SLOW); 
    delay(900); 
    // straightening out 
    Left_F(SLOW); 
    Right_R(STOP+15); 
    delay(1000);
    Stop(); 
    delay(1000); 

    digitalWrite(Blade, HIGH);    // turn blade back on after obstacle is cleared 
    
    
  }
  evading = false; //******************************************************************************************************// if we keep falling into ODET, set this to true to only do it once
  Serial.println("Obstacle avoided"); 
}
//******************************************************************************************************// 
// not implemented due to lack of time 
//******************************************************************************************************// 
/*
void Read_GPS(void)
{ 
  Serial2.begin(9600);
  Serial2.flush(); 
  bool reading = true; 
  while(reading)
  { 
    if (Serial2.available() > 5)
    {
      reading = false; 
    }
    delay(10); 
    Serial.println("waiting for GPS"); 
  }
  int cnt = 0; 
  for (cnt = 0; cnt <6; cnt++)
  {
    heading_STR[cnt] = Serial2.read();
  }
  heading_F = atof(heading_STR); 
  Serial.printf("heading = %f ", heading_F); 
  //Serial.printf("headint str = %s", heading_STR); 
  Serial.println(); 
  Serial2.end(); 
  //delay(1000); 
  /*
  heading_STR = Serial2.readString();
  heading_F = atof(heading_STR); 
  Serial.printf("heading = %f", heading_F); 
  Serial.println(); 
  
   
}
*/
//******************************************************************************************************// 
/*
void Course_Corret(void) 
{ 
  Serial.println("Correcting"); 
  //heading_L = heading_F; 
  //Read_GPS();
  bool Correct = false;  
  L_A_GLOB = 0; 
  R_A_GLOB = 0; 
  while(!Correct) 
  {
    attachInterrupt(Pin_LA, L_A_ISR, RISING);

    attachInterrupt(Pin_RA, R_A_ISR, RISING);
    
    Serial.println("loop reset"); 
    Read_GPS();
    Serial.printf("H_L = %f, H_F = %f", heading_L, heading_F); 
    Serial.println(" "); 
    //heading_L = heading_F; 
    L_A_GLOB = 0; 
    R_A_GLOB = 0; 
    
    if ((heading_L < heading_F)& (abs(heading_L - heading_F) > 7))
    { // need to turn slighlt left 
      while(R_A_GLOB < 7)
      {
        
        Right_F(TURN); 
        Left_R(TURN_STOP);
        Serial.println("Correcting right"); 
      } 
      Right_R(7); 
      Left_F(7);
    }
    else if ((heading_L > heading_F) & (abs(heading_L - heading_F) > 7))
    {
      while(L_A_GLOB < 7)
      {
        
        Left_F(TURN); 
        Right_R(TURN_STOP);
        Serial.println("Correcting right"); 
      } 
      Right_F(7); 
      Left_R(7); 
    }
    else if(abs(heading_L - heading_F) <7)
    { 
      Correct = true; 
      Serial.println("Course correct");
      //delay(1000);
      Right_R(7); 
      Left_R(7); 
    }
      
  }
  detachInterrupt(Pin_LA);

  detachInterrupt(Pin_RA);
}
*/

//******************************************************************************************************// 
// Turn Right Function 
void Turn() 
{ 
  // mod to turn left instead 
  //Serial.println("Turining"); 
  // sets mower to turn, angle not given
  //R_SE_CNT_GLOB = 0; 
  //L_SE_CNT_GLOB = 0; 
  // Right motor forward 
  // Left motor break 
  // try reversing left motor at low power to amplify breaking action 
  //attachInterrupt(Pin_RA, R_A_ISR, RISING); 

  //delay(500);     // to allow coast down 
  attachInterrupt(Pin_RA, R_A_ISR, RISING);  

  

  // turning left forward
  Right_F(TURN); 

  

  // turining right backwards  
  
  Left_R(TURN_STOP); 


  // breaks Right Motor 
  
  // Attach ISRs to A&B channles of right SE here 
  // to monitor correct turing only!!
  //Serial.println("Turining"); 
  /* does a little more than 90 deg on the first turn
   *  then seems to be losing power. 
   *  second turn will be correct, but weaker 
   *  struggles on the 3rd turn 
   *  and is incapable of turining mower on 4th 
   *  not sure of cause, charging battery to be safe 
   */
  R_A_GLOB = 0; 
  //int turncnt = 0;
  bool turning = true;    // loop control  
  //float Ang_x = 0.0; 
  //float Ang_x_i = 0.0;
  
  while(turning)      // L_A_GLOB has 48CPR, so 82/48 wheel rotations should be a 90 deg turn 
  {
    /*    Gyro based turning, uncommented as Gyro sensor fails to provide reliable data 
    
    float Ang_x_i = Ang_x;              // itial angle is last angle 
     
    float CyC_i = ESP.getCycleCount(); //esp_timer_get_time(); // ESP.getCycleCount();  // sys clock at begin 
    imu.read();                         // reads accel and gyro 
    float CyC_F = ESP.getCycleCount(); //esp_timer_get_time(); // ESP.getCycleCount();  // sys clock after imu reading 
    
    float dt = (CyC_F - CyC_i) / (80000000); // time elapsed (1000000); //  / 
   
    Ang_x = Ang_x_i + (imu.g.x - Gyro_Cal)*dt; // curent angle is last angle + deg traveled 
    Serial.printf("X ang = %f", Ang_x); 
    Serial.println(" "); 
    Serial.printf("G_X = %d, G_X_C = %f Gyro_Cal = %f dt = %f", imu.g.x, (imu.g.x - Gyro_Cal), Gyro_Cal, dt);
    Serial.println(" "); 
    
    //Serial.printf("%d ", R_A_GLOB); // for testing 
    //Serial.println("Right CH A");   // for testing 
    //delay(10);              // turining left forward / right back, and cutting rots in half
     
    if ((Ang_x >= 90) | ( Ang_x <= -90))
    { 
      turning = false; 
      Serial.println("Gyro Trigger"); 
       
    }
    */
    Serial.printf("R A = %d", R_A_GLOB); 
    Serial.println(" "); 
    if (R_A_GLOB >= 140)      // from 84 -> 150; 140 -> 155; 84-> 90 -> 105 // 84 should be 90 deg turn  // 130 WORKED WELL BRO LOOK HERE LOOOOOOOOKKKKKKKK 
    {
        
        turning = false; 
        Serial.println("SE trigger"); 
    }
    
  }
  Right_R(10); 

  Left_R(5); 
  

  //delay(3000); 
  
  // inside wheel won't stay locked. trying to turn both wheel now 
  // detach ISRs from channel A&B 
  //detachInterrupt(Pin_RA); 
  detachInterrupt(Pin_RA); 
  Serial.println("turn compleate"); 

  // to stop movement but not incur damage and H-bridge 
  
  
  // Stop(); moving to main  
}
// turing out
//******************************************************************************************************// 
// Go home function 
void Go_Home(void) 
{ 
  int half_dist_rots = ((Leg0*100)/2) * ( 1/Rot_dist);    // rotations to travel to outside of box
  /* mower should end up at the center, pointing the same direction it started 
   *  so we turn 180 ( 90 deg twice), go half the distance of the box widht
   *  turn 90 deg again, go along the outside until we reach our stating point 
   *  and then turn 90 degs again to face the way we did when we started 
   */

  Turn(); // 180 turn 
  Turn(); 
  GoStraight(half_dist_rots); // half distance to reach outside 
  Turn(); // 90 deg turn 
  GoStraight(half_dist_rots); // back to start point 
  Turn(); // algin to face the way we did when we started 
  Stop(); 
  // turn off motors 
  digitalWrite(PinL_1, HIGH); 

  digitalWrite(PinL_2, HIGH);

  digitalWrite(PinR_1, HIGH);

  digitalWrite(PinR_2, HIGH);
}

//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 
// IO setup 
//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 


// ------------------------------------------- //------------------------------------------//

// Method of creating and reading files;
// Start: Create filess
void createFiles(String _file, String content) {
  //
  bool createFile = SPIFFS.exists("/" + _file + ".txt");
  if (createFile) {
    File file = SPIFFS.open("/" + _file + ".txt", "r");
    if (!file) {
      exit(0);
    }
    int s = file.size();

    String data = file.readString();

    file.close();
  } else {
    File file = SPIFFS.open("/" + _file + ".txt", "w");
    file.println(content);
    file.close();
  }
}
// Final: Create Files

// Start Methodo: Set Files of the Status and Update
// Read a String and Save into File
void setStatusFiles(String arquivo, String var) {
  bool createFile = SPIFFS.exists("/" + arquivo + ".txt");
  if (createFile) {
    File file = SPIFFS.open("/" + arquivo + ".txt", "w");
    file.println(var);
    file.close();
  }
}// Final Methodo

/*Set variables of the States and Update*/
/*Read file and return your value*/
String setStatusVar(String _file) {
  bool createFile = SPIFFS.exists("/" + _file + ".txt");
  if (createFile) {
    File file = SPIFFS.open("/" + _file + ".txt", "r");
    String var = file.readStringUntil('\n');
    //String var = file.readString();
    file.close();
    return var;
  }
}
// Final Method;

// ------------------------------------------- //------------------------------------------//

// ------------------------------------------- //------------------------------------------//
// Method for read String containing JSON
String readJson(String strJsonInput, String indexJsonInput) {
  String strJson = strJsonInput;
  String indexJson = indexJsonInput;
  String valueJson = "";
  int cont = 0;
  int cont1 = 0;
  String positionWord = "n";
  char readValue = 'n';
  while (cont < strJson.length()) {
    if (strJson.startsWith(indexJson, cont)) {
      readValue = 's';
      break;
    }
    cont++;
  }
  while (positionWord == "n") {
    if (!strJson.startsWith("\"", (cont + indexJson.length() + 3 + cont1))) {
      valueJson += strJson.charAt(cont + indexJson.length() + 3 + cont1);
      cont1++;
    } else {
      positionWord = "s";
      break;
    }
  }
  if (readValue == 's') {
    return valueJson;
  } else {
    return "false";
  }
}
// ------------------------------------------- //------------------------------------------//

// ------------------------------------------- //------------------------------------------//
// Method write Json
String writeJson(String strJsonInput, String indexJsonInput, String insertJson) {
  String strJson = strJsonInput;
  int lengthstrJson = strJson.length();
  String indexJson = indexJsonInput;
  int lengthIndexJson = indexJson.length();
  String strInsertJson = insertJson;
  String valueJson = "";
  int cont = 0;
  int cont1 = 0;
  String positionWord = "n";
  char readValue = 'n';
  while (cont < strJson.length()) {
    if (strJson.startsWith(indexJson, cont)) {
      readValue = 's';
      break;
    }
    cont++;
  }
  String strInicio = strJson.substring(0, cont + lengthIndexJson + 3);
  while (positionWord == "n") {
    if (!strJson.startsWith("\"", (cont + indexJson.length() + 3 + cont1))) {
      cont1++;
    } else {
      positionWord = "s";
      break;
    }
  }
  String strFinal = strJson.substring(cont + 3 + cont1 + lengthIndexJson, lengthstrJson);
  if (readValue == 's') {
    return valueJson = strInicio + strInsertJson + strFinal;
  } else {
    return "false";
  }
}

//-------------------------------------------- // -----------------------------------------//


void setup() {
  Serial.begin(115200);

  // Start variable:
  strJsonFirebase = "{\"battery\":\" \",\"start_status\":\" \",";
  strJsonFirebase += "\"_0_latitude\":\" \",\"_0_longitude\":\" \",\"_1_latitude\":\" \",\"_1_longitude\":\" \",";
  strJsonFirebase += "\"_2_latitude\":\" \",\"_2_longitude\":\" \",\"_3_latitude\":\" \",\"_3_longitude\":\" \",";

  // Configuration of the .TXT File - Data:
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Creation of files that inform if you are sending or receiving data:
  createFiles("sendReceive", "0");
  createFiles("firebaseReceive", strJsonFirebase);
  createFiles("strJsonFirebase", strJsonFirebase);
  createFiles("clientInput", "0");

  /*Initializes Variables:*/
  sendReceive = setStatusVar("sendReceive");
  firebaseReceive = setStatusVar("firebaseReceive");
  strJsonFirebase = setStatusVar("strJsonFirebase");
  clientInput = setStatusVar("clientInput");

  /*Update variables reveive from firebase*/
  start_status =  readJson(strJsonFirebase, "start_status"); // Received from Firebase
  _0_latitude = readJson(strJsonFirebase, "_0_latitude");  // Received from Firebase
  _0_longitude = readJson(strJsonFirebase, "_0_longitude"); // Received from Firebase
  _1_latitude = readJson(strJsonFirebase, "_1_latitude");  // Received from Firebase
  _1_longitude = readJson(strJsonFirebase, "_1_longitude"); // Received from Firebase
  _2_latitude = readJson(strJsonFirebase, "_2_latitude");  // Received from Firebase
  _2_longitude = readJson(strJsonFirebase, "_2_longitude"); // Received from Firebase
  _3_latitude = readJson(strJsonFirebase, "_3_latitude");  // Received from Firebase
  _3_longitude = readJson(strJsonFirebase, "_3_longitude"); // Received from Firebase


  // Test
  Serial.println("strJsonFirebase: " + strJsonFirebase);

  // Acces Point and SoftAP configuration:
  WiFi.mode(WIFI_AP_STA); //Access Point mode
  WiFi.softAP(ssidEspServer, PassEspServer);    //Password length minimum 8 char

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  //connect to WiFi
  WiFi.begin(ssidWiFiLocalSet, passwordSenhaWiFiLocal);

  byte contWiFi = 0;
  while ((WiFi.status() != WL_CONNECTED) and contWiFi < 30) {
    delay(500);
    Serial.print(".");
    contWiFi++;
  }

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.println("[WIFI] Connecting");
  delay(500);
  Serial.println(WiFi.localIP());

  //Set your Firebase info
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  //Enable auto reconnect the WiFi when connection lost
  Firebase.reconnectWiFi(true);

  //Start Server
  server.begin();

  // Semaphore and Tasks creation
  myMutex = xSemaphoreCreateMutex();
  if (myMutex != NULL) {
    //create a task
    xTaskCreatePinnedToCore(
      TaskWiFi,   /* Task function. */
      "TaskWiFi",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      3,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(500);

    //create a task
    xTaskCreatePinnedToCore(
      TaskLocalControl,   /* Task function. */
      "TaskLocalControl",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      3,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */
    delay(500);

    //create a task
    xTaskCreatePinnedToCore(
      TaskRemoteControl,   /* Task function. */
      "TaskRemoteControl",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      4,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */
    delay(500);

  }
  // ------------------------------------------- NAV_SETUP()//------------------------------------------//
  
    // digital output pins 
    // constant control logic output 
    pinMode(PinL_1, OUTPUT);

    pinMode(PinL_2, OUTPUT);

    pinMode(PinR_1, OUTPUT);

    pinMode(PinR_2, OUTPUT);

    digitalWrite(PinL_1, HIGH); 

    digitalWrite(PinL_2, HIGH);

    digitalWrite(PinR_1, HIGH);

    digitalWrite(PinR_2, HIGH);

    

 
    // PWM pins 
    ledcAttachPin(12,1); //( PinL_F, L_Channel);left forward

    ledcAttachPin(13,2); //( PinL_R, R_Channel); left backwards

    ledcAttachPin(26,3); // PinR_R  right reverse 

    ledcAttachPin(27,4); // PinR_F right forward 

    

 

    ledcSetup(1,5000,8);//( L_Channel, freq, resolution);

    ledcSetup(2,5000,8);//( R_Channel, freq, resolution);

    ledcSetup(3,5000,8);//

    ledcSetup(4,5000,8);//


    // Right SE indicator channel 
    pinMode(Pin_RSE, INPUT); 
    
    // ISR on rising edge of interupt channel right 
    attachInterrupt(Pin_RSE, R_SE_ISR, RISING);  

    // SE channel A inputs for L & R
    pinMode(Pin_RA, INPUT);

    pinMode(Pin_LA, INPUT);
    // Left SE Indicator channel 

    pinMode(Pin_LSE, INPUT); 

    pinMode(O_ALERT, INPUT); 

    pinMode(Blade, OUTPUT); 

    //attachInterrupt(O_ALERT, ODET, RISING); 
    
    // ISR on rising edge of interupt channel left 
    //attachInterrupt(Pin_LSE, L_SE_ISR, RISING); 
    // Note Ch- A on left and right has 48 CPR 
    // indicator channel has one ping per compelated rotation 

    // Read GPS here for home location 

    Serial.begin(115200);

    //attachInterrupt(Pin_LA, L_A_ISR, RISING);

    //attachInterrupt(Pin_RA, R_A_ISR, RISING);

    // IMU init, set and calibarte uncommen ted due to unreliable IMU sensors 

    //Wire.begin(); // begins I2C 

    //imu.init(); // init Gyro and Accel 

    //imu.enableDefault(); // and set to default 

    //int cnt = 0; 
    //imu.read(); 
    //float Sum = 0.0; 
    /*
    for (cnt = 0; cnt < 50; cnt++); 
    { 
      imu.read(); 
      Sum = Sum + imu.g.x; 
      
    }
    */
    //Gyro_Cal = imu.g.x; //Sum/50;    // X axis gyro reading when mower is still 
}
 // ------------------------------------------- nav_main_loop()//------------------------------------------//
 
// Task WiFi
void TaskWiFi( void * pvParameters ) {
  for (;;) {
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    if (WiFi.status () == WL_CONNECTED) {
      printGlobal("wificonnect !!!!");
    }
    else {
      printGlobal("falhou !!!!");
      WiFi.reconnect ();
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}


// Task TaskLocalControl
void TaskLocalControl( void * pvParameters ) {
  for (;;) {
    /*Initializes the Variables:*/
    printGlobal("Local-Control");
    xSemaphoreTake(myMutex, portMAX_DELAY);
    sendReceive = setStatusVar("sendReceive");

    xSemaphoreGive(myMutex);

    if (sendReceive.toInt() == 0) { // Connection Selection
      WiFiClient client = server.available();   // Listen for incoming clients
      String header;
      char c;
      String OpcaoAcao;
      if (client) {                             // If a new client connects,
        ////Serial.println("New Client.");          // print a message out in the //Serial port
        String currentLine = "";                // make a String to hold incoming data from the client
        if (client.connected()) {            // loop while the client's connected
          //if (client.connected()) {
          while (client.available()) {             // if there's bytes to read from the client,
            c = client.read();             // read a byte, then
            ////Serial.write(c);                    // print it out the //Serial monitor
            header += c;
            //Serial.print(header);
            if ((c == '\n') and (header.length() > 49)) {

              // Save the last data received from the slave in a file
              xSemaphoreTake(myMutex, portMAX_DELAY);
              setStatusFiles("clientInput", header);
              xSemaphoreGive(myMutex);

            }

          }

          // Send the last data received from Firebase to the slave
          client.print("{\"start_status\":\" " + start_status + " \",");
          client.print("\"_0_latitude\":\" " + _0_latitude + " \",");
          client.print("\"_0_longitude\":\" " + _0_longitude + " \",");
          client.print("\"_1_latitude\":\" " + _1_latitude + " \",");
          client.print("\"_1_longitude\":\" " + _1_longitude + " \",");
          client.print("\"_2_latitude\":\" " + _2_latitude + " \",");
          client.print("\"_2_longitude\":\" " + _2_longitude + " \",");
          client.print("\"_3_latitude\":\" " + _3_latitude + " \",");
          client.print("\"_3_longitude\":\" " + _3_longitude + " \",");
          client.flush();
          client.stop();
        }
      }
      if (header.length() > 10) {

        // Save the last data received from Slave
        xSemaphoreTake(myMutex, portMAX_DELAY);
        start_status = readJson(header, "start_status"); //Test
        latitude = readJson(header, "latitude"); //Test
        longitude = readJson(header, "longitude"); //Test
        battery = readJson(header, "battery"); //Test
        clientInput = header;
        xSemaphoreGive(myMutex);

        printGlobal(battery);//Test
        printGlobal(receiveCar);//Test
      }
      header = "";
      setStatusFiles("sendReceive", "1");
    }
    vTaskDelay(110 / portTICK_PERIOD_MS); // original valor: 110
  }
}

// Task TaskRemoteControl - Send and Receive Data from Firebase
void TaskRemoteControl( void * pvParameters ) {
  for (;;) {
    if (sendReceive.toInt() == 1) {
      // Send Firebase
      xSemaphoreTake(myMutex, portMAX_DELAY);
      if (Firebase.setString(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/battery", battery)) 
      {
        //Success
        Serial.println("Set string data success");
      } else {
        //Failed?, get the error reason from fbdo
        Serial.print("Error string setString, ");
        Serial.println(fbdo.errorReason());
      }

      // Get Firebase
      xSemaphoreGive(myMutex);

      // Get Firebase /cfp1Uk5Vykbdgh44KVlkyFCyE4c2/start_status
      if (Firebase.getBool(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/start_status"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        start_status = String(fbdo.boolData());
        strJsonFirebase = writeJson(strJsonFirebase, "start_status", start_status);
        xSemaphoreGive(myMutex);
        printGlobal("start_status: ");
        printGlobal(start_status);
      }
      else {
        //Failed?, get the error reason from fbdo
        Serial.print("Error getting start status, ");
        Serial.println(fbdo.errorReason());
      }

      // Get Firebase custom_lawns/test1/locations/mapPositions/0/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/0/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _0_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_0_latitude", _0_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_0_latitude: ");
        printGlobal(_0_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/0/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _0_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_0_longitude", _0_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_0_longitude: ");
        printGlobal(_0_longitude);
      }
      // Get Firebase custom_lawns/test1/locations/mapPositions/1/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/1/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _1_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_1_latitude", _1_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_1_latitude: ");
        printGlobal(_1_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/1/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _1_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_1_longitude", _1_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_1_longitude: ");
        printGlobal(_1_longitude);
      }
      // Get Firebase custom_lawns/test1/locations/mapPositions/2/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/2/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _2_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_2_latitude", _2_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_2_latitude: ");
        printGlobal(_2_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/2/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _2_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_2_longitude", _2_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_2_longitude: ");
        printGlobal(_2_longitude);
      }
      // Get Firebase custom_lawns/test1/locations/mapPositions/3/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/3/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _3_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_3_latitude", _3_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_3_latitude: ");
        printGlobal(_3_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/3/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _3_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_3_longitude", _3_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_3_longitude: ");
        printGlobal(_3_longitude);
      }


      // Save data receive from firebase
      setStatusFiles("strJsonFirebase", strJsonFirebase);
      printGlobal(strJsonFirebase);
      xSemaphoreTake(myMutex, portMAX_DELAY);
      setStatusFiles("sendReceive", "0");
      xSemaphoreGive(myMutex);
    }
    //vTaskDelay(110 / portTICK_PERIOD_MS);
    vTaskDelay(110 / portTICK_PERIOD_MS);
  Leg0 = calcDist(_0_longitude,_0_latitude,_1_longitude,_1_latitude); //Call the distance and bearing calculation function
  Leg1 = calcDist(_1_longitude,_1_latitude,_2_longitude,_2_latitude); //Call the distance and bearing calculation function
  Leg2 = calcDist(_2_longitude,_2_latitude,_3_longitude,_3_latitude); //Call the distance and bearing calculation function
  Leg3 = calcDist(_3_longitude,_3_latitude,_0_longitude,_0_latitude); //Call the distance and bearing calculation function

  }
}

double calcDist(String longitude_1, String latitude_1, String longitude_2, String latitude_2){ //This is a haversine based distance calculation formula
  //convert String coordinates to double
  lonR1 = longitude_1.toDouble();
  lonR2 = longitude_2.toDouble();
  latR1 = latitude_1.toDouble();
  latR2 = latitude_2.toDouble();
  //This portion converts the current and destination GPS coords from decDegrees to Radians
  lonR1 = lon1*(PI/180);
  lonR2 = lon2*(PI/180);
  latR1 = lat1*(PI/180);
  latR2 = lat2*(PI/180);
 
  //This portion calculates the differences for the Radian latitudes and longitudes and saves them to variables
  dlon = lonR2 - lonR1;
  dlat = latR2 - latR1;
 
  //This portion is the Haversine Formula for distance between two points. Returned value is in KM
  a = (sq(sin(dlat/2))) + cos(latR1) * cos(latR2) * (sq(sin(dlon/2)));
  e = 2 * atan2(sqrt(a), sqrt(1-a)) ;
  d = (R * e);
  

  Serial.println();
  Serial.print("Distance to destination(M): ");
  //Serial.println(a);
  //Serial.println(e);
  Serial.println(d,6);//(d,6)
  Serial.println();
 
  
}
void loop() {
  while (start_status == "1"){
    /* We are given leg distances in M, and calculate the max number of loops we need. 
   *  each start of this loop, we incriment the leg number, as we are on a new leg. 
   *  once leg number == 3 (ie 4 legs compleate) we reset leg num and incriment loop num. 
   *  We are now on the next full loop. 
   *  We itterate untill we have compleated all loops, when we fall into an infinte wait loop ( for now) 
   *  
   *  EACH LOOP OF THIS FUNCTION GOES STRAIGHT FOR ONE LEG AND THEN COMPLEATES A (HOPEFULLY) 90 DEG TURN 
   */

   /* ODET: Add ISR form OD-MCU alert pin and read distance side 
    *  move to approriate side, OD-MCU alert pin can set low once obstacle no longer in sight 
    *  then move read distance forward + some error margin 
    *  then rotate other wheel to point back at original course, and move same distance we originally moves 
    *  rotate to straight and proceed 
    *  
    *  May not work as we might need SE data from both wheels 
    */
   // can check course from GPS here to validate 
   digitalWrite(Blade, HIGH);     // turn on blade 
   
 

    // put your main code here, to run repeatedly:
    // If we have ran the full track 
    if (LoopNum >= LoopsNeeded) 
    { 
      // NOTE: need to modify this with RTB and sleep code 
      Stop();     // stop the mower 
      digitalWrite(Blade,LOW); // turn off blade 
      Go_Home(); 
      
      
      while(1)
      {
        delay(1000);    // and wait in this loop forever 
        Serial.println("Compleate, in wait loop"); 
      }
    }

    
    if (LegNum == 4)
    { 
      // if LegNum == 3 we have done 4 legs, and need to restart our rectangle, with one mower width less than before 
      LegNum = 0;   // resetin leg num  
      LoopNum++;    // incrimenting loop num 
      Serial.printf("Loop number %d Compleated", LoopNum);
      delay(1000);  
    }
    Serial.printf("Going Straight on Leg %d", LegNum);
     
     
    // go straight the distance of the current leg(in cm) - one half-track width per compleated loop 

    R_SE_CNT_GLOB = 0; 
    L_SE_CNT_GLOB = 0;
    int rots = floor((LegAr[LegNum]*100)/Rot_dist) - LoopNum; 
    GoStraight( rots); 
    //delay(2000); 
    if(start_status == "0"){
      Stop();
      break;
    }
    // after we've gone a leg, we stop the mower
    Stop();
    delay(1000); // and give it time to come to a stop 
     
    // reset SE counts,  
    R_SE_CNT_GLOB = 0; 
    L_SE_CNT_GLOB = 0;
    // and incriment to the next leg number 
    LegNum++;  

    // here we turn 
    // while the distance traveled by the left wheel is less than what is needed to turn 90 degrees: run turn 
    // NOTE: currently only set up to handle integer wheel turns (using channel I, not A&B), 
    // we also don't know where the wheel is as we enter this function, so we may need to use A&B here to get an exact count 
      
    Turn();
    //delay(1000);
    
    // after we've compleated our turn, we stop again 
    Stop();
    // Reset rotation counts 
    R_SE_CNT_GLOB = 0; 
    L_SE_CNT_GLOB = 0; 
    // delay to allow for coast 
    delay(1000); 
  }
}
