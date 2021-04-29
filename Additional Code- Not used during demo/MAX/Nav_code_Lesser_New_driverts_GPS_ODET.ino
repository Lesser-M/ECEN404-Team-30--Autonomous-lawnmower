/*********************************************************************************/ 
/*********************************************************************************/ 
/*********************************************************************************/
/* COde modified for new motor control logic 
 * Basic navigation test code, with option for course correction 
 *  uses shaft encoders, PWM motor driver boad 
 *  
 *  written by max  
 *  
 *  initial pin assignments and setup for motor controll pins (PWM and function pins) 
 *  as well as set up of PWM done by Josh. All other functionality written by max lesser 
 *  
 *  Addtions/ comments: 
 *  - commented out all serial print calls
 *      - not needed for opperationl testing. frees up resources 
 *      
 *  - added course correction ability 
 *      - in go straight and turn function, boolen check for correction varible 
 *        if correction is asserted, go straight checks CH- A counts, and slows down one wheel 
 *        if the difference between the 2 wheels if more then 1/12th rotation ( 4 counts) 
 *        
 *      - in break function, if one wheel has stopped but not the other, incrimentatlly increase  
 *        reverse thrust on the wheel that has not stopped  
 *        
 *  - modified turn function  
 *      -- turn function now sets wheels to coast after set number of rotations (ie turn) is compleate) 
 *      
 *  - modified stop function 
 *      - now uses loop, and keeps asserting reverse power until there is no change in CH-A values for 0.1 second 
 *      - sets motors to coast after stop is compleate 
 *      
 * 
 * 
 * TODO: 
 *  - validate CPR -> Validated at 48 
 *  - validate / test STOP PWM value -> Stop PWM setting is good 
 *  - validate / test TURN PWM value -> working on this one, inside wheel does lock up, had some slipage in the outside turing wheel 
 *  but some extra weight did that. We do have some play in how the shaft is attached to the motor, and i need to validate the actual rotation distance 
 *  
 *  
 *        
 */ 

/*********************************************************************************/ 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
// Includes 
#include <Wire.h>         // I2C 
#include <LIS3MDL.h>      // Compass
#include <LSM6.h>         // Gyro and Accel 

LSM6 imu;                 // Gyro and accel instance 

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/


// Pin labels 

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

const uint8_t O_ALERT = 35;     // Front Obstacle alert pin, DEFINE ON PIC AND CONNECT 
   
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
  
  char heading_STR [6];     // read buffer for heading  
  
  float heading_F = 0.0;    // current heading 

  float heading_L = 0.0;    // last heading 

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

bool OBSTACLE = false; 

bool evading = false; 


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

void IRAM_ATTR ODET()
{ 
  // if OD alert pin sets high, set the OBSTACLE bool to high 
  OBSTACLE = true; 
  //Serial.println("Found it"); 
  //delay(2000); 
}

//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 
// NAV FUNCTIONS 
//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 


//******************************************************************************************************// 

// sets left motor to go forward 
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
//******************************************************************************************************// 
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
void Right_C(void)
{
  ledcWrite(3,255);
  ledcWrite(4,255);
  //digitalWrite(PinR_F, HIGH);
  //digitalWrite(PinR_R, HIGH);
  //Serial.println("right motor coast"); 
  
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
    delay(100); 
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
  */
   
}
//******************************************************************************************************// 
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

//******************************************************************************************************// 
// Go straight function
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

//******************************************************************************************************// 
void Clear_Obstacle(void)
{ 
  // Obstacle avoidance setup 
  // First thing we need to do is stop dead 
  /*default to turn to right, so that we can keep the obstacle on out 
   * left, and in view of the left side sensors. 
   */
  bool TimeBased = false; 
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
    // turing out 
    Left_F(SLOW); 
    Right_R(Stop); 
    delay(200);
    // going past  
    Left_F(SLOW); 
    Right_F(SLOW); 
    delay(1000); 
    // turning back towards original path 
    Left_R(STOP); 
    Right_F(SLOW); 
    delay(400); 
    // coming back 
    Left_F(SLOW); 
    Right_F(SLOW); 
    delay(1000); 
    // straightening out 
    Left_R(STOP); 
    Right_F(SLOW); 
    delay(200);
    
    
  }
  evading = false; //******************************************************************************************************// if we keep falling into ODET, set this to true to only do it once
  Serial.println("Obstacle avoided"); 
}
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
    
    delay(10);    // used as NOP instruction here I feel like i should take this out, but I'm leaving it here cause that's what i last ran at 
    //Serial.printf("On rot num %d ", R_SE_CNT_GLOB); 
    //Serial.println("Going"); 
       //delay(10);    // used as NOP instruction here 
    Serial.printf("On rot num %d ", R_SE_CNT_GLOB); 
    Serial.println("Going"); 
    if ( ((OBSTACLE) | (O_ALERT == HIGH)) & (!evading) )
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
    /*
    
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
//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 
// IO setup 
//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 

void setup() {

   

    // put your setup code here, to run once:

 
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
    
    // ISR on rising edge of interupt channel left 
    //attachInterrupt(Pin_LSE, L_SE_ISR, RISING); 
    // Note Ch- A on left and right has 48 CPR 
    // indicator channel has one ping per compelated rotation 

    // Read GPS here for home location 
    attachInterrupt(O_ALERT, ODET, RISING); 

    Serial.begin(115200);

    //attachInterrupt(Pin_LA, L_A_ISR, RISING);

    //attachInterrupt(Pin_RA, R_A_ISR, RISING);

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
    //Read_GPS(); 
    //heading_L = heading_F; 
    

    

    



   
    

}

 
//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 
// MAIN 
//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 

void loop() {

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
    /* function used in testing, disregrad
    delay(5000); 
    Left_F(70); 
    delay(5000); 
    Left_C(); 
    delay(5000); 
    Left_R(70); 
    delay(5000); 
    Left_C();

    Right_F(70); 
    delay(5000); 
    Right_C(); 
    delay(5000); 
    Right_R(70); 
    delay(5000); 
    Right_C(); 
    delay(5000); 
    */
    //Turn(); 
    //delay(1000); 
    //Turn(); 
    //delay(1000); 
    
    //GoStraight(10); 
    //delay(1500); 
    //Stop(); 
    //delay(1500); 
    //Turn();
    //delay(1500);
    /*
    Left_R(75);  
    delay(3000); 
    Right_R(75); 
    delay(3000); 
    Right_F(75); 
    delay(3000); 
    Left_F(75); 
    delay(3000);
    */ 
    /*
    Turn(); 
    delay(1000); 
    Turn(); 
    delay(1000); 
    Turn(); 
    delay(1000); 
    Turn(); 
    delay(1000); 
    //Course_Corret(); 
    */
   
   
    // If we have ran the full track 
    if (LoopNum >= LoopsNeeded) 
    { 
      // NOTE: need to modify this with RTB and sleep code 
      Stop();     // stop the mower 
      while(1)
      {
        delay(1000);    // and wait in this loop forever 
        //Serial.println("Compleate, in wait loop"); 
      }
    }

    
    if (LegNum == 4)
    { 
      // if LegNum == 3 we have done 4 legs, and need to restart our rectangle, with one mower width less than before 
      LegNum = 0;   // resetin leg num  
      LoopNum++;    // incrimenting loop num 
      //Serial.printf("Loop number %d Compleated", LoopNum);
      delay(1000);  
      Course_Corret(); 
    }
    //Serial.printf("Going Straight on Leg %d", LegNum);
     
     
    // go straight the distance of the current leg(in cm) - one half-track width per compleated loop 

    R_SE_CNT_GLOB = 0; 
    L_SE_CNT_GLOB = 0;
    int rots = floor((LegAr[LegNum]*100)/Rot_dist) - LoopNum; 
    GoStraight(rots); 
     
    
    // after we've gone a leg, we stop the mower
    Stop();
   
     
    // reset SE counts,  
    
    // and incriment to the next leg number 
    LegNum++;  

    // here we turn 
    // while the distance traveled by the left wheel is less than what is needed to turn 90 degrees: run turn 
    // NOTE: currently only set up to handle integer wheel turns (using channel I, not A&B), 
    // we also don't know where the wheel is as we enter this function, so we may need to use A&B here to get an exact count 
    //delay(100);   
    Turn();
    
    
    // after we've compleated our turn, we stop again 
    Stop();
    // Reset rotation counts   

    // and then restart 
    //
    
  
}
