/* Nav code for new motors, with obstacle dection , No wifi implementd. 
 * Nav code demoed is based on this version 
 * 
 * The intital declaration and setup of motor control pins (PinL1, PinL2, PinR1, PinR2, PinL and PinR)  
 *  and PWM pins in this code was from a code snippet done by Josh Samaniego. as well as the setup of the 2 PWM channels  
 *  All other functionality, including remaing setup, ISRs, turn, stop, straight and obstacle aviodance function 
 *  as well as Main function written by Max Lesser. 
 *  
 *  Code relies on input form PIC32 -ODET MCU for obstacle detection. 
 *  Reads shaft encoders form left and right motors. 
 *  And sends control signal to left and right motors 
 *  ( with functionality for blade, and reading GPS from PIC to be added if possible) 
 */
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// PIN ASSIGNEMNTS for motor controll
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

const uint8_t PinL  = 12;  // 12 corresponds to GPIO12 

const uint8_t PinR  = 13;  // 13 corresponds to GPIO13 

const uint8_t PinL_in1  = 2;  // 12 corresponds to GPIO15 

const uint8_t PinL_in2  = 15;  // 13 corresponds to GPIO0 

const uint8_t PinR_in3  = 18;  // 12 corresponds to GPIO4 

const uint8_t PinR_in4  = 4;  // 13 corresponds to GPIO2 


///////////////////////////////////////////////////////
//setting PWM properties

const uint32_t freq = 5000;

const uint8_t L_Channel = 1; // note channel 0 and 1 share frequencies

const uint8_t R_Channel = 2; // note channel 2 and 3 share frequencies

const uint8_t resolution = 8;


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// PIN ASSIGNEMNTS fos shaft encoders 
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

const uint8_t R_SE_A = 22;         // Right SE channel A

const uint8_t R_SE_B = 0;       // Rigth SE chanel B 

const uint8_t L_SE_A = 23;       // Left SE channel A 

const uint8_t L_SE_B = 0;     // Left SE channel B 

const uint8_t Pin_RSE = 26;   // Right SE indicator channel GPIO 26 m.lesser 4/3/21

const uint8_t Pin_LSE = 27;   // Left SE indicator channel GIPO 27 m.lesser 4/3/21

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// PIN ASSIGNEMNTS for OD-MCU interface 
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t O_ALERT = 35;     // Front Obstacle alert pin, DEFINE ON PIC AND CONNECT 

const uint8_t Side_Det = 34;    // Side obstacle detect, DEFINE ON PIC AND CONNECT 

const uint8_t UTX = 1; 

const uint8_t URX = 3;        // UART0 TX and RX pins I hope 


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VERIABLES for shaft encoder readings and obstacle detection 
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Global Vars for se indicator counts  
int L_SE_CNT_GLOB = 0;  // left se channel I ( or X) counter  

int R_SE_CNT_GLOB = 0;  /// right se channel I ( or X) counter 

int R_A_GLOB = 0;       // right se channel A counter 

int R_B_GLOB = 0;       // right se channel B counter 

int L_A_GLOB = 0;       // left se channel A counter  

int L_B_GLOB = 0;       // left se channel B counter 

bool OBSTACLE = false;  // bool to tell us if we have an obstacle 

bool evading = false;     // bool to determine if we are already in obstacle evasion patter. 
                              // to keep the function from being called in go straight, and becoming recursive. 

 


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// NAVIGATION CONSTANTS 
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

const double Rot_dist = 47.625; // liner distnace travled in one shaft rotation 


 // each shaft roation corresponds to ~ 3/4 of a wheel rotation, where the wheel diameter is ~ 7.75 inches
 // Wheel circumference is ~ 63.5 cm, meaining each shaft rotation is ~ 47.625 cm => 48 cm ? 

 const double trackWidth = 15;    // track width of mower in cm ( guessed at 15 cm for now) 

 // 90 deg turn rotation distance for left wheel 

 const double TurnDist = 79.8;    // rotation of left wheel needed to compleate 90 deg turn

 const uint8_t CPR = 48;          // Counts per revoltuion for A&B channels of shaft encoders 

 ///////////////////////////////////////////////////////////////////////////////////////////////
 // SPEED SETTINGS 
////////////////////////////////////////////////////////////////////////////////////////////////
 const uint8_t SLOW = 64;         // Slow speed PWM setting 

 const uint8_t MED = 128;         // Medium speed PWM setting 

 const uint8_t FAST = 200;        // Fast speed PWM setting

 const uint8_t  STOP = 0;         // setting PWM for stop 

// get legs from W-MCU 

double Leg0 = 6;      // Leg 0 distance in M 

double Leg1 = 6;      // Leg 1 distance

double Leg2 = 6;      // Leg 2 distnace 

double Leg3 = 6;      // Leg 3 distance 

double LegAr[4] = {Leg0, Leg1, Leg2, Leg3}; 



const uint8_t LoopsNeeded = floor ((LegAr[0]*100)/Rot_dist); // (Leg0*100)/(2*trackWidth));    // total number of loops needed, based on leg0 in the group

int LegNum = 0;     // tracks what leg we are on 

int LoopNum = 0;    // Tracks which loop around the yard we're on

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// ISRs for shaft encoders and Obstacle alert 
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// ISR m.lesser 4/3/21
//////////////////////////////////////////////
// LEFT SE NOT TRIGGERING new SE to be added 
/////////////////////////////////////////////
// ISR to incriment left SE cnt 

void IRAM_ATTR L_SE_ISR() {     
  L_SE_CNT_GLOB ++; 
  // on pin 27 
  
  
}

// ISR for A&B channle left side 
void IRAM_ATTR L_A_ISR() { 
  L_A_GLOB++; 
  
}

void IRAM_ATTR L_B_ISR() { 
  L_B_GLOB++; 
  
}

// ISR to incriment right SE cnt 
void IRAM_ATTR R_SE_ISR() {
  R_SE_CNT_GLOB++;
  // on pin 26 
  
}

// ISR for A&B channle right side 
void IRAM_ATTR R_A_ISR() { 
  R_A_GLOB++; 
  
}

void IRAM_ATTR R_B_ISR() { 
  R_B_GLOB++; 
  
}
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
// ISR FOR OD-MCU alert pin 
// ISR on rising edge 
// set Bool for Obstacle detected and exit
// bool OBSTACLE  = true 
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

void IRAM_ATTR ODET()
{ 
  // if OD alert pin sets high, set the OBSTACLE bool to high 
  OBSTACLE = true; 
}


/*************************************************************************************/
/*************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// NAVIGATION FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/*************************************************************************************/
/*************************************************************************************/
// Go straight function 
void GoStraight(int Rotations)
{
  // sets output to go straight, for dist 
  // straight for left 
  //Serial.printf("Going Straight for %f", dist); 
  // should be set just once in setup 
  //digitalWrite(PinL_in1, HIGH);

  //digitalWrite(PinL_in2, LOW);
   
  //Reverse for right motor

  //digitalWrite(PinR_in3, LOW);

  //digitalWrite(PinR_in4, HIGH);

  // at Medium speed, see above 

  ledcWrite(1,MED);

  ledcWrite(2,MED);
  
  //int Rotations = floor(dist/Rot_dist); // num of shaft rotations needed to travel dist;  
  
   
    while ((R_SE_CNT_GLOB < Rotations))
    {
      delay(10);    // used as NOP instruction here 
      Serial.printf("On rot num %d ", R_SE_CNT_GLOB); 
      Serial.println("Going"); 
      if (((OBSTACLE) | ((O_ALERT == HIGH)) & (!evading)))
      {
        Serial.println("Begin Obstacle Evasion Now"); 
        Clear_Obstacle(); 
      }
    }
  

}

void Clear_Obstacle()
{ 
  // Obstacle avoidance setup 
  // First thing we need to do is stop dead 
  /*default to turn to right, so that we can keep the obstacle on out 
   * left, and in view of the left side sensors. 
   */
  
  Stop(); 
  evading = true; 
  OBSTACLE = false; // need to reset here to not fall into evasion repeatedly, i think                                                 // check here// 
  delay(1000);    // making sure we are actually stoped/ and or time to coast down 
  // saving context as we enter obstacle avoidance 
  int R_Context = R_SE_CNT_GLOB; 
  int L_Context = L_SE_CNT_GLOB; 
  // Attaching Interupts. 
  // Detached at end of this function 
  attachInterrupt(R_SE_A, R_A_ISR, RISING);
  
  //attachInterrupt(R_SE_B, R_B_ISR, RISING);
  
  attachInterrupt(L_SE_A, L_A_ISR, RISING); //HW not working only gives ~ 3CPR, will try to use and correct for that 
  
  //attachInterrupt(L_SE_B, L_B_ISR, RISING); B channles not used 
  
  ////////////////////////////////////////////////////////////////////////////////
  // reading OD info from pic 
  // OD string should have following format" 
  // dd,s <distance,Side> where distanceis in CM, <99 cm, and side is either R,L,C 
  // ALSO:: SET ALERT PIN AND SEND STRING ONLY AT DISTANCE OF 50CM 
  // Baud Rate, Protocal ( need to verify with pic) TX and RX Pins (Need to assign) 
  /*
  Serial2.begin(9600, SERIAL_8N1, UTX, URX); 
  char ODreadout[4]; 
  char dist[2]; 
  char side; 
  int cnt = 0; 
  for (cnt = 0; cnt <4; cnt++) 
  {
    ODreadout[cnt] = Serial2.read(); 
  }
  for(cnt = 0; cnt <2; cnt++) 
  {
    dist[cnt] = ODreadout[cnt];
  }
  int dist_cm = atoi(dist);     // distance in cm as integer value 
  side = ODreadout[3];          // charachter representing what side we are on 

  // Turning Out of obstacle 
  Serial.printf("Obstacle on %s at %d", side, dist_cm); 
  Serial.println("beginning evasion");
  */ 
  //char side = 'L';  
  L_A_GLOB = 0; 
  R_A_GLOB = 0; 
  
    //if(side == 'L')
    //{
  Serial.println("Turning out rigth"); 
      // Spin left wheel until we are no longer looking at the obstacle 
      // pins should be set just once in setup, as we will not be reversing 
  //digitalWrite(PinL_in1, HIGH);

  //digitalWrite(PinL_in2, LOW);
      
  ledcWrite(1,SLOW);    // slow speed on left motor  
  while ((O_ALERT == HIGH) | ( L_A_GLOB < floor((TurnDist/Rot_dist)*CPR)))
  {
    // keep turning out until we hav cleared the obstacle, or turned by 90 degrees 
    delay(10);  // NOP statement 
    
  }
   // } 
   
   /*
     else 
     {
       // Spin Right wheel until we are no longer looking at the obstacle 
       Serial.println("Turning out Left"); 
      
        digitalWrite(PinR_in3, LOW);

        digitalWrite(PinR_in4, HIGH);
      
        ledcWrite(2,200);
     } 
     */
  

  Stop(); 
  // Keeping track of how much we stepped out 
  float SideStepRots = 0.0; 
  // note, we use A channel here, but it does not matter, from how we're driving the wheels we know we're goind forward
  // and CH A & B both have 500CPR, just want an absoulte value for wheel rotations 
  //if (side == 'L')
 // {
  SideStepRots = (L_A_GLOB/CPR);     // L-CH-A CPR holds CPRS for both left and right A channel 
  //}
  //else 
  //{ 
   // SideStepRots = (R_A_GLOB/CPR);     // Same as for left side 
  //}

  // Distance we need to move at new angle to clear obstacle 
  // removed dist_cm 
  float ClearDist = 200; // 80 + 63.5 + 20;      // dist_cm is reported distance to obstacle, 63.5 (cm) is length from Array to end of mower, 20 is safety factor; 
  // num rotations associated with this distance 
  int ClearRots = ceil(ClearDist/Rot_dist);   // taking ceil here just in case 

  // clearing obstacle 
  Serial.println("clearing"); 
  GoStraight(ClearRots); 


  //if (side == 'L') 
  //{
  ledcWrite(2,SLOW);    // turining right wheel slowly 
  Serial.println("Straightening out to left");   
  while(R_A_GLOB < CPR*SideStepRots)    // should be integer  
  {
    // we are upconverting SideStepRots here, cause it is a constant, just in case L_A_GLOB changed slightly 
    // Now we want to spin Right wheel to straighen out 
    // pins set in setup 
    delay(10); 
    //digitalWrite(PinR_in3, LOW);

    //digitalWrite(PinR_in4, HIGH);
      
    
  //} 
  }
    
  //}
  /*
  else 
  { 
    Serial.println("straightening out to right"); 
    while(L_A_GLOB < SideStepRots*3)
    { 
      // See case above 
      digitalWrite(PinL_in1, HIGH);
  
      digitalWrite(PinL_in2, LOW);
        
      ledcWrite(1,200); 
      
    }
    */
  Stop(); 
  
  int ParaRots = 0; 

  
  if (Side_Det == HIGH) 
  { 
    int tempRots = R_SE_CNT_GLOB; 
    Serial.println("Obstacle on side, need to keep goinf straight"); 
    ledcWrite(2,SLOW);
    ledcWrite(1,SLOW); 
    while(Side_Det == HIGH)
    { 
      // if we are on a track parallel to the obstacle, but we are still detecting it on the side, we want to go straigh untill we stop detecting it on the side 
      //digitalWrite(PinL_in1, HIGH);
    
      //digitalWrite(PinL_in2, LOW);
          
       delay(10); 
  
      //digitalWrite(PinR_in3, LOW);
    
      //digitalWrite(PinR_in4, HIGH);
          
      
    }
    ParaRots = R_SE_CNT_GLOB - tempRots;    // number of rotations we traveled parallel to the obstacle 
    
  }
  Stop(); 


  // Turn Back into path step 
  //if (side == 'L')
  //{ 
  R_A_GLOB = 0; 
  Serial.println("Turning in left"); 
  ledcWrite(2,SLOW);
  while (R_A_GLOB < SideStepRots * 500)
  { 
    delay(10); 
    //digitalWrite(PinR_in3, LOW);

    //digitalWrite(PinR_in4, HIGH);
      
     
  }
  Stop(); 
  //}
  /*
  else 
  {
    L_A_GLOB = 0; 
    Serial.println("Turning in rigth"); 
    while (L_A_GLOB < SideStepRots * 3)
    { 
      digitalWrite(PinL_in1, HIGH);
  
      digitalWrite(PinL_in2, LOW);
        
      ledcWrite(1,200); 
    }
    Stop(); 
  }
  */
  Serial.println("returning to original path"); 
  GoStraight(ClearRots); // retracing step from above 
  
  // straigtening out on path 
   //if (side == 'L')
  //{ 
  L_A_GLOB = 0; 
  Serial.println("straightening back out to right"); 
  ledcWrite(1,SLOW);
  while (L_A_GLOB < SideStepRots * CPR)
  { 
    delay(10); 
    //digitalWrite(PinL_in1, HIGH);

    //digitalWrite(PinL_in2, LOW);
      
     
  }
  Stop(); 
  //}
  /*
  else 
  {
    R_A_GLOB = 0; 
    Serial.println("straightening out to left"); 
    while (R_A_GLOB < SideStepRots * 500)
    { 
      digitalWrite(PinR_in3, LOW);
  
      digitalWrite(PinR_in4, HIGH);
        
      ledcWrite(2,200); 
    }
    Stop(); 
  }
  */
   
  
  // we are now clear of the obstacle, and pointing back into the original direction 

  // Finnally we need to restore the shaft encoder contexts
   // NOW need to adjust linear distance 
     // Formula: 
     // (side_rots/(rots_for_90_deg) * (pi/2) : angle in rads that we sheered out -> phi 
     // clearance rots: distance we traveled at that angle (ie hypothenouse of our triangle) 
     // SideStep_Rots: distance we traveled parallel to out path (may be 0 for small obstacles) 
     // Liner distance along original path (Lin_Dist_ = 2*((cos(phi))*clerance_rots) + SideStep_Rots
     //                                      sheer out +sheer back in     + parallel dist 

  float phi = ( ( (SideStepRots /500) *Rot_dist)/ TurnDist) *(3.141/2);       // angle in rads that we turned out 
  int Lin_Dist = (2*cos(phi)*ClearRots) + ParaRots;                      // not taking into account wheel circumference, as it sould drop out as a factor 
  R_SE_CNT_GLOB = R_Context + Lin_Dist; 
  L_SE_CNT_GLOB = L_Context + Lin_Dist; 

  // detach interupts here 
  detachInterrupt(R_SE_A); 

  detachInterrupt(L_SE_A); 
  ///////////////////////////////////////////////////////////////////////////////////////////
  // NOTE 
  // L-SE-A only runs about 3CPR when it should be 500. have adjusted code to use 3 as CPR count. 
  // Do not know how reliable the 3 number is. 
  // May need to compleatly remove left SE from program 
  
  //OBSTACLE = false;     // we have cleared the obstacle, and can set it's bool to false  
  evading = false; 
  Serial.println("Obstacle avoided"); 
}
/* void clear_obstacle()
 *  {
 *  /////////////////////////////////////////////////////////////////
 *  Basic Idea 
 *  we need to store the SE context for both wheels and stop imedialty. 
 *  once we have that and have read where the obstacle is. 
 *  we turn the approriate wheel to move away from the obstacel. 
 *  WE KEEP TRACK OF HOW MANY ROTS WE TURNED OUT. 
 *  we then move forward by a distance = mower length + distance to obstacle. 
 *  wife then rotate the opposite wheel by the same turn out rotations as above, to put the mower on a path parallel to the original path. 
 *  we then check side detect sesor. if it sees something, we go straight until it doesn't ( may need to do some extra bc field of view of sensor) 
 *  after that (or if side detect is clear) 
 *  we turn back towards our original path, by the side step number of rotations stored above. 
 *  we then spin the outside wheel that same number to point straight again. 
 *  finally we take the saved context and add to it the liner distance (rotations) we traveled keep distance accurate, and return this to go straight function. 
 *  see below for formula 
 *  
 *  also may need to attach A and B channel interupts to keep more accurate track. 
 *  
 *    R_context = R_SE_CNT_GLOB; 
 *    L_context = L_SE_CNT_GLOB; 
 *    stop(); 
 *    read_uart_buffer(); 
 *    // parse for obstacle side
 *    delay(1000); 
 *    while(alert_pin == HIGH) 
 *    {
 *      if(obstacle_left)
 *      {
 *        // spin left wheel slowly 
 *        // "turn to right, however that ends up working out 
*       } 
*       else 
*       {
*       // spin right wheel slowly 
*       // "turn to left however that ends up working out 
*       } 
*     } 
*     clearance_dist_rots = bstace_distance+mowerlength0/Rot_dist; 
*     GoStraight(clearance_dist_rots))
*     if (obstacle_left) 
*     { 
*       // right rots = L_SE_CNT_GLOB-L_context // WE NEED THIS VALUE, FIND THIS FIRST 
*       // spin right wheel slowly 
*       // "turn back to straight 
*       // spin right wheel same number of we spun left above 
*        
*     }
*     else 
*     {
*       // spin left wheel slowly 
*       // "turn back to straight 
*       // spin left wheel same number of we spun left above 
*       // left_rots = R_SE_CNT_GLOB-R_context;  
*     } 
*     SideStep_ROTS_i = R_SE_CNT_GLOB; 
*     while (side_obstacle_alert_pin == HIGH) 
*     { 
*       // go straight slowly 
*       // ie spin both wheels till we're clear 
*       // 
*     } 
*     SIdeSTep_rots = R_SE_CNT_GLOB - SIdeStep_Rotsi; // rotations we traveled parallel to our original path, is at all 
*     if (obstacle_left) 
*     { 
*       // turn right wheel slowly 
*       // right_rots = right_rots from above
*       GoStraight(clerance_dist_rots) 
*       // turn left wheel slowly to straighen out again 
*       // left_rots = right rots from above 
*     } 
*     else 
*     { 
*       // turn left wheel slowly 
*       // left_rots = left_rots; 
*       GoStraight(clearnce_dist_rots); 
*       // turn right wheel to straigthten out 
       // right_rots = left rots; 
     } 
     // NOW need to adjust linear distance 
     // Formula: 
     // (side_rots/(rots_for_90_deg) * (pi/2) : angle in rads that we sheered out -> phi 
     // clearance rots: distance we traveled at that angle (ie hypothenouse of our triangle) 
     // SideStep_Rots: distance we traveled parallel to out path (may be 0 for small obstacles) 
     // Liner distance along original path (Lin_Dist_ = 2*((cos(phi))*clerance_rots) + SideStep_Rots
     //                                      sheer out +sheer back in     + parallel dist 
     R_SE_CNT_GLOB = R_context + Lin_Dist; 
     L_SE_CNT_GLOB = L_context + Lin_Dist; 
     
     obstacle = false; // need to reset detection bool 
   } 
*/ 

// Stop Function 
void Stop()
{
  Serial.println("Stopping"); 
  // stops mower 

  ledcWrite(1,STOP);

  ledcWrite(2,STOP);

  delay(100); // to give time to stop                                                                     // may need to adjust this// 

 /*
  // breaks Left Motor 
  digitalWrite(PinL_in1, LOW); 

  digitalWrite(PinL_in2, LOW);

  
  // breaks Right Motor 
  digitalWrite(PinR_in3, LOW);

  digitalWrite(PinR_in4, LOW);
  */
}


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

  // reversing left wheel for additonal break boost 
  //digitalWrite(PinL_in1, LOW);

  //digitalWrite(PinL_in2, HIGH);
  
  attachInterrupt(L_SE_A, L_A_ISR, RISING); 

  L_A_GLOB = 0; 

  ledcWrite(1,STOP);
  
  ledcWrite(2,STOP);
  
  //digitalWrite(PinR_in3, LOW); 

  //digitalWrite(PinR_in4, HIGH);


  // breaks Right Motor 
  
  // Attach ISRs to A&B channles of right SE here 
  // to monitor correct turing only!!
  Serial.println("Turining"); 
  ledcWrite(1,SLOW);
  while(L_A_GLOB < floor((TurnDist/Rot_dist) * CPR))
  {
    delay(10);  
  }
  ledcWrite(1,STOP);
  
  detachInterrupt(L_SE_A); 
  
  // detach ISRs from channel A&B 
}

void setup() {

  /*********************************************************************************************
   * *******************************************************************************************
   * TODO and NOTES 
   * 
   * Code will fall into obstacle detection function, and seems to take appropriate action 
   * Obstical was premantly placed infront of sensor array, which would cause unexpected behavior 
   * Need to test with obstacle that can actually be avioded. 
   * But it does fall into the function when opperating at most basic extent 
   * Minimize use of delays and char arrays 
   * Removed UART and just hard set mower to turn to right and obstacle at fixed 80 cm, which is OD alert distance 
   * 
   * also left SE is fucked and only does ~ 3CPR instead of 500, no I channel 
   * 
   * Side Detect not implemented on Pic, as we need to find an approriate pin 
   */

   

    // put your setup code here, to run once:

    
    // digital output pins 
    // reconfigure for new motor driver 
    pinMode(PinL_in1, OUTPUT);

    pinMode(PinL_in2, OUTPUT);

    pinMode(PinR_in3, OUTPUT);

    pinMode(PinR_in4, OUTPUT);

    

 
    // PWM pins 
    ledcAttachPin(12,1); //( PinL, L_Channel);

    ledcAttachPin(13,2); //( PinR, R_Channel);

 

    ledcSetup(1,5000,8);//( L_Channel, freq, resolution);

    ledcSetup(2,5000,8);//( R_Channel, freq, resolution);

    //////////////////////////////////////////////////////
    // SEs 
    
    // Right SE 
    //indicator channel 
    pinMode(Pin_RSE, INPUT);

    // These have ISRs, however we only use them for obstacle avoidance and possibly in turns 
    // in order to not have ~6K interupts/second 
    // Right CH A 
    pinMode(R_SE_A, INPUT); 

    //attachInterrupt(R_SE_A, R_A_ISR, RISING); 

    // Right CH B 
    pinMode(R_SE_B, INPUT); 
    
    // ISR on rising edge of interupt channel right 
    attachInterrupt(Pin_RSE, R_SE_ISR, RISING);

    

    // Left SE
    
    //Indicator channel 
    pinMode(Pin_LSE, INPUT); 

    // These have ISRs, however we only use them for obstacle avoidance and possibly in turns 
    // in order to not have ~6K interupts/second 
    // Left CH A 
    pinMode(L_SE_A, INPUT); 

    //attachInterrupt(L_SE_A, L_A_ISR, RISING); 
    // LE SE A, B channel only run ~ 3cpr, should be 500CPR 

    // Left CH B 
   // pinMode(L_SE_B, INPUT); 

    
    // ISR on rising edge of interupt channel left 
    attachInterrupt(Pin_LSE, L_SE_ISR, RISING); 
    // Left SE I channel not working 

    //

    // Read GPS here for home location 


    // Pin and ISR for ODET alert signal 
    pinMode(O_ALERT, INPUT); 

    attachInterrupt(O_ALERT,ODET, RISING); 

    // side detection pin
    pinMode(Side_Det, INPUT); 
    // commented out, suspect char writing causes 0x1c exception, illegal load 
    //Serial2.begin(9600, SERIAL_8N1, UTX, URX); 

    Serial.begin(115200);\
    

    
}

 

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
   // can check course from GPS here to validate 
   /*
   while(1)
   {
    Serial.println("F"); 
    if (O_ALERT == HIGH) 
    { 
      Serial.println("Obstacle within 80 cm"); 
      delay(1000); 
    }
    else if (OBSTACLE == true) 
    { 
      Serial.println("Obstacle !!!!"); 
      OBSTACLE = false; 
    }
    */
    
    /*
    if ( 4 <= Serial2.available())
    {
      for (cnt = 0; cnt <4; cnt++) 
      {
        ODreadout[cnt] = Serial2.read(); 
      }
      for(cnt = 0; cnt <2; cnt++) 
      {
        dist[cnt] = ODreadout[cnt];
      }
      int dist_cm = atoi(dist);     // distance in cm as integer value 
      side = ODreadout[3];          // charachter representing what side we are on 
    
      // Turning Out of obstacle 
      Serial.printf("Obstacle on %s at %d", side, dist_cm); 
      Serial.println("beginning evasion"); 
      delay(1000); 
      */ 
    
   // delay(500); 
   //}
  
 

    
    // If we have ran the full track 
    if (LoopNum >= LoopsNeeded) 
    { 
      // NOTE: need to modify this with RTB and sleep code 
      Stop();     // stop the mower 
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

    // and then restart 
}
