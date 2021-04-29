/*********************************************************************************/ 
/*********************************************************************************/ 
/*********************************************************************************/
/* Basic Nav code written for old motor system. 
 * 
 * Basic navigation test code, with option for course correction 
 *  uses shaft encoders, PWM motor driver boad 
 *  
 *  written by max lesser. 
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
 *  - validate CPR 
 *  - validate / test STOP PWM value 
 *  - validate / test TURN PWM value 
 *  
 *  
 *        
 */ 

/*********************************************************************************/ 
// Pin labels 

const uint8_t PinL  = 12;  // 12 corresponds to GPIO12 

const uint8_t PinR  = 13;  // 13 corresponds to GPIO13 

const uint8_t PinL_in1  = 2;  // 12 corresponds to GPIO15 

const uint8_t PinL_in2  = 15;  // 13 corresponds to GPIO0 

const uint8_t PinR_in3  = 18;  // 12 corresponds to GPIO4 

const uint8_t PinR_in4  = 4;  // 13 corresponds to GPIO2 

const uint8_t Pin_RSE = 26;   // Right SE indicator channel GPIO 26 m.lesser 4/3/21

const uint8_t Pin_LSE = 27;   // Left SE indicator channel GIPO 27 m.lesser 4/3/21

const uint8_t Pin_LA = 32;        // left ch a se pin 
 
const uint8_t Pin_RA = 33;     // right ch A pin     
   
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



const uint8_t CPR = 48;         // counts per revoltuion on ch A                                                // check/validate this number 

//**************************************************// 
// speed settings for different opperations 

const uint8_t STOP = 30;        // base pwm setting for stopping 

const uint8_t TURN = 170;       // PWM setting for turning. to be applied to only one motor while the other is stopped. 

const uint8_t SLOW = 64;        // slow speed setting 

const uint8_t MED = 150;        // medium speed setting 

const uint8_t FAST = 225;       // fast speed setting 

                                                     //******************************************************************************************************// 
                                                     //******************************OI LOOK HERE YA BAT ****************************************************// 
                                                     //******************************************************************************************************// 

bool correction = false;         // set to true to correct wheel speed to go straight. If things fuck up, set to false 


 // each shaft roation corresponds to ~ 3/4 of a wheel rotation, where the wheel diameter is ~ 7.75 inches
 // Wheel circumference is ~ 63.5 cm, meaining each shaft rotation is ~ 47.625 cm => 48 cm ? 
 const double Rot_dist = 47.625; // liner distnace travled in one shaft rotation                                // check this number                                          

 const double trackWidth = 15;    // track width of mower in cm ( guessed at 15 cm for now) 

 // 90 deg turn rotation distance for left wheel 

 const double TurnDist = 79.8;    // rotation of left wheel needed to compleate 90 deg turn                    // check this too 
 //******************************************************************************************************// 

/*********************************************************************************/ 
  // Nav variables 
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

double Leg0 = 6;      // Leg 0 distance in M 

double Leg1 = 6;      // Leg 1 distance

double Leg2 = 6;      // Leg 2 distnace 

double Leg3 = 6;      // Leg 3 distance 

double LegAr[4] = {Leg0, Leg1, Leg2, Leg3}; 



const uint8_t LoopsNeeded = floor ((LegAr[0]*100)/Rot_dist); // (Leg0*100)/(2*trackWidth));    // total number of loops needed, based on leg0 in the group


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
  // on pin 27 
  //Serial.printf(" rotation %d ", L_SE_CNT_GLOB); 
  //Serial.println("Left Wheel interupt Triggered"); 
  
}

// ISR to incriment right SE cnt 
void IRAM_ATTR R_SE_ISR() {
  R_SE_CNT_GLOB++;
  // on pin 26 
  //Serial.printf(" rotation %d ", R_SE_CNT_GLOB); 
  //Serial.println("Right wheel interupt Triggered"); 
}

void IRAM_ATTR R_A_ISR() {
  R_A_GLOB++;
  // on pin 26 
  //Serial.printf(" rotation %d ", R_SE_CNT_GLOB); 
  //Serial.println("Right wheel interupt Triggered"); 
}


void IRAM_ATTR L_A_ISR() {
  L_A_GLOB++;
  // on pin 26 
  //Serial.printf(" rotation %d ", R_SE_CNT_GLOB); 
  //Serial.println("Right wheel interupt Triggered"); 
}

//******************************************************************************************************// 
//******************************************************************************************************// 
//******************************************************************************************************// 
// NAV FUNCTIONS 
//******************************************************************************************************// 
//******************************************************************************************************// 
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
  
  digitalWrite(PinL_in1, LOW);

  digitalWrite(PinL_in2, HIGH);
   
  //Reverse for right motor

  digitalWrite(PinR_in3, HIGH);

  digitalWrite(PinR_in4, LOW);

  // at full speed 

  ledcWrite(1,MED);

  ledcWrite(2,MED);
  //int Rotations = floor(dist/Rot_dist); // num of shaft rotations needed to travel dist;  
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
    
    delay(10);    // used as NOP instruction here 
    //Serial.printf("On rot num %d ", R_SE_CNT_GLOB); 
    //Serial.println("Going"); 
  }

  // coasting both wheels after we've compleated the straigh leg 

  digitalWrite(PinL_in1, LOW);

  digitalWrite(PinL_in2, LOW);

  digitalWrite(PinR_in3, LOW);

  digitalWrite(PinR_in4, LOW);

  
  if(correction)
  {
    detachInterrupt(Pin_LA);

    detachInterrupt(Pin_RA);
  }

}
//******************************************************************************************************// 
// Stop Function 
void Stop()
{
  //Serial.println("Stopping"); 
  attachInterrupt(Pin_LA, L_A_ISR, RISING);

  attachInterrupt(Pin_RA, R_A_ISR, RISING);

  R_A_GLOB = 0;   // resetting ch a encoder counts  

  L_A_GLOB = 0; 

  L_last = 0;   // hold last encode value on channel A, so check if we've actully stopped 

  R_last = 0; 
  
  // stops mower 
  // breaking by applying low power reverse 
  // breaks Left Motor 
  ledcWrite(1,STOP);
  
  digitalWrite(PinL_in1, HIGH); 

  digitalWrite(PinL_in2, LOW);

  

  
  // breaks Right Motor 
  ledcWrite(2,STOP);
  
  digitalWrite(PinR_in3, LOW);

  digitalWrite(PinR_in4, HIGH);

   

  ind = 1;              // incrementally boosts reversing power 

  while(!stopped | (ind < 10)) 
  { 
    L_last = L_A_GLOB;    // last SE- ch A count becomes current CH- A count for both wheels 

    R_last = R_A_GLOB; 

    delay(100);           // delay by 0.1 seconds. assumption is that if we have not moved at least 1/48th of a rotation in 0.1 seconds we are stopped 

    if ((L_last == L_A_GLOB) & (R_last == R_A_GLOB))                                                      // possibly include error bound here? 
    { // if both left and right wheel haven't moved more than 1/48th of a rotation in the last 0.1 seconds
      // mower is stopped 
      stopped = false; 
     
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
  
  digitalWrite(PinL_in1, LOW);

  digitalWrite(PinL_in2, LOW);
  
  digitalWrite(PinR_in3, LOW);

  digitalWrite(PinR_in4, LOW);

  
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
  attachInterrupt(Pin_LA, L_A_ISR, RISING);  
  

  // turning left 
  digitalWrite(PinL_in1, LOW);

  digitalWrite(PinL_in2, HIGH);

  ledcWrite(1,TURN);

  ledcWrite(2,STOP);

  // breaking right 
  
  digitalWrite(PinR_in3, LOW); 

  digitalWrite(PinR_in4, HIGH);


  // breaks Right Motor 
  
  // Attach ISRs to A&B channles of right SE here 
  // to monitor correct turing only!!
  //Serial.println("Turining"); 
  L_A_GLOB = 0; 
  
  while(L_A_GLOB < 82)      // L_A_GLOB has 48CPR, so 82/48 wheel rotations should be a 90 deg turn 
  {
    delay(10);  
  }
  // detach ISRs from channel A&B 
  //detachInterrupt(Pin_RA); 
  detachInterrupt(Pin_LA); 
  
  Stop(); 
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
    pinMode(PinL_in1, OUTPUT);

    pinMode(PinL_in2, OUTPUT);

    pinMode(PinR_in3, OUTPUT);

    pinMode(PinR_in4, OUTPUT);

    

 
    // PWM pins 
    ledcAttachPin(12,1); //( PinL, L_Channel);

    ledcAttachPin(13,2); //( PinR, R_Channel);

 

    ledcSetup(1,5000,8);//( L_Channel, freq, resolution);

    ledcSetup(2,5000,8);//( R_Channel, freq, resolution);


    // Right SE indicator channel 
    pinMode(Pin_RSE, INPUT); 
    
    // ISR on rising edge of interupt channel right 
    attachInterrupt(Pin_RSE, R_SE_ISR, RISING);  

    // SE channel A inputs for L & R
    pinMode(Pin_RA, INPUT);

    pinMode(Pin_LA, INPUT);
    // Left SE Indicator channel 

    pinMode(Pin_LSE, INPUT); 
    
    // ISR on rising edge of interupt channel left 
    //attachInterrupt(Pin_LSE, L_SE_ISR, RISING); 

    // Read GPS here for home location 

    Serial.begin(115200);



   
    

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
   // can check course from GPS here to validate 
   
 

    // put your main code here, to run repeatedly:
    
    

    
      
    
     
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
    }
    //Serial.printf("Going Straight on Leg %d", LegNum);
     
     
    // go straight the distance of the current leg(in cm) - one half-track width per compleated loop 

    R_SE_CNT_GLOB = 0; 
    L_SE_CNT_GLOB = 0;
    int rots = floor((LegAr[LegNum]*100)/Rot_dist) - LoopNum; 
    GoStraight( rots); 
     
    
    // after we've gone a leg, we stop the mower
    Stop();
   
     
    // reset SE counts,  
    
    // and incriment to the next leg number 
    LegNum++;  

    // here we turn 
    // while the distance traveled by the left wheel is less than what is needed to turn 90 degrees: run turn 
    // NOTE: currently only set up to handle integer wheel turns (using channel I, not A&B), 
    // we also don't know where the wheel is as we enter this function, so we may need to use A&B here to get an exact count 
      
    Turn();
    
    
    // after we've compleated our turn, we stop again 
    Stop();
    // Reset rotation counts   

    // and then restart 
}
