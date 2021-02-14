/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "ObjectDetect.h"



/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

int Read_Front_Array(struct Reading R1, struct Reading R2, struct Reading R3, struct Reading R4, struct Reading R5)
    { /* Front Array read function 
       * Reads 5 sensors in front array, for pins as labeled in pin assignment 
       * below. Calculates ToF for each sensor, and then distance in cm 
       * for speed of sound = 340m/s 
       * 
       * Using Timer 3, with 40ms period, 1562500 hz Frequncy 
       * giving resolution of ~ 25ns 
       * 
       * PROCESS: 
       * we send a common 10us trigger pulse to all sensors, via a common pin (as labled below) 
       * we then enter a loop, where once a sensors echo pin sets high we 
       * read the timer value into a start variable for that sensor 
       * once we have a start varible for all sensors we enter the next while loop 
       * here we test for each echo pin to reset low, and read the timer value at 
       * that time into a end time varibel for each sensor. Once we have readings 
       * from all 5 sensors we break out of the loop. 
       * the difference between start and end can be converted to one way 
       * travel distance using: 
       * Dist = ((senDif) / (1562)) * (1/1000) [s/us] *(100/1) [cm/m] *340[m/s] *0.5
       *      = senDif * 0.010883 
       * 
       * note on US opperation: 
       * US expects 10 us trigger pulse to trigger pin 
       * Time Of Flight is given by high time of echo pin. 
       */  
        
        // start and finish varibles to hold timer counts for the different sensors 
        // as indicated in varible name 
        uint16_t S1_Start = 0; // tracks start time 
        uint16_t S1_Finish = 0; // tracks end time 
        
        uint16_t S2_Start = 0; // tracks start time 
        uint16_t S2_Finish = 0; // tracks end time 
        
        uint16_t S3_Start = 0; // tracks start time 
        uint16_t S3_Finish = 0; // tracks end time 
        
        uint16_t S4_Start = 0; // tracks start time 
        uint16_t S4_Finish = 0; // tracks end time 
        
        uint16_t S5_Start = 0; // tracks start time 
        uint16_t S5_Finish = 0; // tracks end time
        
       
        bool end = false; // bool to track if we have our end reading 
        
        // setting echo pins as inputs 
        GPIO_RC2_InputEnable(); // S5
        GPIO_RC4_InputEnable(); // S4
        GPIO_RB8_InputEnable(); // S2
        GPIO_RB10_InputEnable(); // S1
        GPIO_RD2_InputEnable(); // S3

        
        // block to send trigger pulse on pin RB14
        // timer counts to 62500 in 40ms 
        GPIO_RB12_OutputEnable(); // common trigger
        GPIO_RB12_Set(); 
        DelayUs(10); 
        GPIO_RB12_Clear(); 
   
     

        // THIS PART WORKS 
        
//        while (!end)
//        {
//            if (GPIO_RC2_Get() == 1)
//            {
//                S1_Start = TMR3_CounterGet(); 
//                while (!end)
//                {
//                      
//                //start = true; 
//               
//            
//            //DelayUs(10); 
//                    if (GPIO_RC2_Get() == 0) 
//                    {
//                        S1_Finish = TMR3_CounterGet(); 
//                        end = true; 
//                    }
//                }
//            }
//        }
        // used to control While loops to read start and end tmr values respecivly 
        int SenStart = 0; // counts number of sensors that have start tmr value 
        int SenFinish = 0; // counts # sensors that have end tmr value 
        // these bools ensure that we only take one start/end value per timer. 
        // ie that we do not override previous values 
        // start bools 
        bool S1S = false; 
        bool S2S = false;
        bool S3S = false;
        bool S4S = false;
        bool S5S = false;
        // end (Finish) bools )
        bool S1F = false; 
        bool S2F = false;
        bool S3F = false;
        bool S4F = false;
        bool S5F = false;
        /* Reading loops 
         * we have 2 nested loops. The first one itterates until we have 
         * start values for all sensors, controlled by SenStart count 
         * we check each sensor in order, if it's pin has set high (ie echo ToF 
         * signal has started) we read into start time AND set the check bool.
         * this ensures we only read the start time for each sensor once, 
         * incase we need to loop more than once waiting for another sensor. 
         * as all sensor are triggered from the same pin, this is unlikely to happen 
         * for start time BUT!!!! may happen for finish time, as different 
         * sensor can take much longer to finish their reading, depending on distances. 
         * Proccess for end time is similar to start time, except we have SenFinish count 
         * and SXF bool to check if we already have a stop time for any sensor 
         */
        while (!end)
        {
            if ((GPIO_RB10_Get() == 1)&(!S1S))
            {   // S1 
                S1_Start = TMR3_CounterGet(); 
                SenStart ++; 
                S1S = true;
            } 
            if ((GPIO_RB8_Get() == 1)&(!S2S))
            {   //S2
                S2_Start = TMR3_CounterGet(); 
                SenStart++; 
                S2S = true; 
            } 
            if ((GPIO_RD2_Get() == 1)&(!S3S))
            {   //S3
                S3_Start = TMR3_CounterGet(); 
                SenStart++; 
                S3S = true; 
            } 
            if ((GPIO_RC4_Get() == 1)&(!S4S))
            {   //S4
                S4_Start = TMR3_CounterGet(); 
                SenStart++; 
                S4S = true; 
            } 
            if ((GPIO_RC2_Get() == 1)&(!S5S))
            {   //S5
                S5_Start = TMR3_CounterGet(); 
                SenStart++; 
                S5S = true; 
            }
            if(SenStart == 5) // we have start readings from all, can begin looking for ends 
            {       
                while (!end)
                {
                    if ((GPIO_RB10_Get() == 0)&(!S1F)) 
                    {   //S1
                        S1_Finish = TMR3_CounterGet(); 
                        SenFinish++; 
                        S1F = true; 
                    }
                    if ((GPIO_RB8_Get() == 0)&(!S2F))
                    {   //S2
                        S2_Finish = TMR3_CounterGet(); 
                        SenFinish ++; 
                        S2F = true; 
                        
                    }
                    if ((GPIO_RD2_Get() == 0)&(!S3F))
                    {   //S3
                        S3_Finish = TMR3_CounterGet(); 
                        SenFinish ++;
                        S3F = true; 
                    }
                    if ((GPIO_RC4_Get() == 0)&(!S4F))
                    {   //S4
                        S4_Finish = TMR3_CounterGet(); 
                        SenFinish++; 
                        S4F = true; 
                    }
                    if ((GPIO_RC2_Get() == 0)&(!S5F))
                    {   //S5
                        S5_Finish = TMR3_CounterGet(); 
                        SenFinish++; 
                        S5F = true;
                    }
                    if (SenFinish == 5)
                    {   // we now have 5 end readings and can break loop 
                        end = true; 
                    }    
                }
            }
        }
   
        // timer ticks for ToF from sensors 
        int dif_S1 = S1_Finish-S1_Start; // ToF for sensor 1 relative to timmer 3 
        int dif_S2 = S2_Finish-S2_Start; // ToF for sensor 2 relative to timmer 3
        int dif_S3 = S3_Finish-S3_Start; // ToF for sensor 3 relative to timmer 3
        int dif_S4 = S4_Finish-S4_Start; // ToF for sensor 4 relative to timmer 3
        int dif_S5 = S5_Finish-S5_Start; // ToF for sensor 5 relative to timmer 3
        
        // calculating distance from timer ticks, see function description for formula  
        float distance1 = (dif_S1)*0.01088; // ToF in ms * Vsound in M/s * (1/1000)[s/ms]*(100[cm/M])
        float distance2 = dif_S2 * 0.01088; 
        float distance3 = dif_S3 *0.01088; 
        float distance4 = dif_S4 * 0.01088; 
        float distance5 = dif_S5 * 0.01088; 
        
        if (distance1 <=400)
        {
            R1.dist = distance1; 
        }
        else 
        {
            R1.dist = 0.0; 
        }
        if (distance2 <=400)
        {
            R2.dist = distance2; 
        }
        else 
        {
            R2.dist = 0.0; 
        }
        if (distance3 <=400)
        {
            R3.dist = distance3; 
        }
        else 
        {
            R3.dist = 0.0; 
        }
        if (distance4 <=400)
        {
            R4.dist = distance4; 
        }
        else 
        {
            R4.dist = 0.0; 
        }
        if (distance5 <=400)
        {
            R5.dist = distance1; 
        }
        else 
        {
            R5.dist = 0.0; 
        }
        R1.senNum = 1; 
        R2.senNum = 2; 
        R3.senNum = 3; 
        R4.senNum = 4; 
        R5.senNum = 5; 
        //float distance = distance1+distance2+distance3+distance4+distance5;
        detect(R1,R2,R3,R4,R5);
        return 0; 
    }

int detect(struct Reading F1, struct Reading F2, struct Reading F3, struct Reading F4, struct Reading F5)
{
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
    CONVENTIONS AND IMPORTANT REFERENCES USED IN THIS CODE (later to be ammended with high level summar)
    For calculation of obstacles 4 intermediate variables are used


    CONVENTION 
    angle given with normal line as obstacle desription is referenced such that an obstacle to the right of the 
    center of the mower is reported with a negative angle, while objects to the left are given with a postive angle 
    the angle itself is referanced to the normal line ( ie an obsacle directly infront of the mower would be reported 
    with an angle of 90 deg. while obstacles moving away from dead center have decreasing magnitude of angle 
    lower bound to be established )
    */
    


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //KNOWN ISSUES //KNOWN ISSUES //KNOWN ISSUES //KNOWN ISSUES //KNOWN ISSUES //KNOWN ISSUES //KNOWN ISSUES //KNOWN ISSUES 
    // need to classify multiple objects as area objects
    //
    // Establish max/min values and check for those? currentyly only grouped by region, do we need aditional grouping/validation? 

    // simulating sensor readings, to be replaced with actual readings when implemented 

    // What when we only have one distance reading in a resoltuion zone
    
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // DUMMY INPUTS 
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /*
    struct Reading R1; 
    R1.dist = 61.2; //150.0; //61.2; 
    R1.senNum = 1; 

    struct Reading R2; 
    R2.dist = 61.6; //61.6; 
    R2.senNum = 2; 

    struct Reading R3; 
    R3.dist = 100.0; //;// 100.0; // 
    R3.senNum = 3; // 

    struct Reading R4; 
    R4.dist = 99.0;// 99.0; // 
    R4.senNum = 4; // 

    struct Reading R5; 
    R5.dist = 98.7369; //153.0; // 98.7369; // 100.0; 
    R5.senNum = 5; 
     */ 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////BEGIN DECLARATION OF VARIBLES /////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //
    ////
    ////// struct arrays holding readings at various stages of the sorting process 
    ////
    //
    struct Reading O[5] = {F1,F2,F3,F4,F5}; // sensor readings
    //// structs for sorted readings 
    // by distance: 
    struct Reading O0[5] = {}; // Readings between D1 and Do 
    struct Reading O1[5] = {}; // Readings greater than D1 
    struct Reading O2[5] = {}; // Readings greater than D2 
    struct Reading O3[5] = {}; // Readings greater then D3
    
    // for processing sensors w. readings in the same region
    struct Reading Odet[5] = {}; // Readings resolving the same obstacle 

    // array holding detected obstacles, with Normal angle and distance 
    struct Obstacle detected[5] = {}; 

   //
   ////
   ////// Internal count and tracking varibles used in sorting loops and validation procedures 
   ////
   //
   
    int cnt = 0; // counts for sorting loops 
    int cnt2 = 0; // count for sorting loops 

    int cnt3 = 0; // loop count to move elements from area Arrays into detected object array 
    int cntDet = 0; // count of sensors with valid readings in the given detection range 
  
    //
    ////
    ////// Distances defining different boundary regions, where point obstacles can be resolved with different numbers of sensors 
    ////// not including error bounds, those are added in the applicable sorting loops themselves 
    ////
    //
    float D0 = 32.54; // minimum distance to resolve object w 2 sensors 
    float D1 = 65.09; // minimum distance to reolve object w 3 sensors
    float D2 = 97.64; // minimum distance to resolve object w 4 sensors 
    float D3 = 130.19; // minimum distance to resolve object w 5 sensors
 
   //
   ////
   ////// Variables to hold results or intermediate values needed to calcualte results, to be passed to calcualtion functions 
   ////
   //
    float Dadj; // Adjacent distance, intermediate step to result 
    float Dist; // distance from mower to object, result parameter 
    float angleRef; // angle between line from refence sensor to object and front of mower, intermediate value 
    float angleNormal; // angle between front of mower and line from cneter of mower to object
 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////END DECLARATION OF VARIBLES /////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    ////
    ////
    // Loops to sort readings according to boundary region they fall into 
    ////
    ////
    // if we start missing detections check these region bounds  
    
    for (cnt = 0; cnt <5; cnt++)
    { // possible objects further than D3 
        if (O[cnt].dist >= D3)
       {
           O3[cnt2] = O[cnt];
           cnt2++; 
       } 
    }
    cnt = 0;
    cnt2 = 0;  
    for (cnt = 0; cnt <5; cnt++)
    { // possible objects between D3 and D2 including some error bound overlap 
        if ( (O[cnt].dist >= D2) & (O[cnt].dist <= (D3 + maxdiff)))
       {
           O2[cnt2] = O[cnt];
           cnt2++; 
       } 
    }
    cnt = 0; 
    cnt2= 0; 
    for (cnt = 0; cnt <5; cnt++)
    { // possible objects between D2 and D1 
        if ((O[cnt].dist >= D1) & (O[cnt].dist <= (D2 + maxdiff) ))
       {
           O1[cnt2] = O[cnt];
           cnt2++; 
       } 
    }
    cnt = 0; 
    cnt2 = 0; 
    for (cnt = 0; cnt <5; cnt++)
    { // possible objects between D0 and D1  
        if ( (O[cnt].dist >= D0) & ( O[cnt].dist <= (D1 + maxdiff)))
       {
           O0[cnt2] = O[cnt];
           cnt2++; 
       } 
    }



    ////
    ////
    // Grouping readings for same object from boundary regions, beging with closest 
    ////
    ////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* old
    Repeate this process for the other detection zones. 
    Pass Odet array to angle and distance solve functions outlined below 

    remaining issues, what when we have multiple sensor readingfs in a zone refering to different objects? 
    What about area objects? 
    */
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Variables to track up to 3 different obstacles 
    float angleRef1 = 0.0; 
    float Dist1 = 0.0; 
    float Dadj1 = 0.0; 
    float angleNormal1 = 0.0;
    // obstacle 2 
    float angleRef2 = 0.0; 
    float Dist2 = 0.0; 
    float Dadj2 = 0.0; 
    float angleNormal2 = 0.0;
    // obstacle 3 
    float angleRef3 = 0.0; 
    float Dist3 = 0.0; 
    float Dadj3 = 0.0; 
    float angleNormal3 = 0.0;

    // detected obstacles in different areas
    // any number of sensor readings in an area will be treated as 
    // an area obsatcle 

    struct Obstacle object0;
    struct Obstacle object1;
    struct Obstacle object2;
    struct Obstacle object3;
    // initialzing distance so we can find empties later 
    object0.distance = 0; 
    object1.distance = 0; 
    object2.distance = 0; 
    object3.distance = 0; 


    
    ////// 
    ////// NOTE FOR ALL Odet Arrays, senseros in array are ordered left to right when looking in dirve direction 
    //////

    ////
    // For obstacles falling into region D0 
    ////
    if (O0[0].dist != 0)
    { 
        for (cnt3 = 0; cnt3 <5; cnt3 ++)
        {
            if(O0[cnt3].dist != 0)
            {
                Odet[cnt3]= O0[cnt3]; // sorting obstacles from O0 into detection array 
                cntDet ++; 
            }
        }
        // change these block to directly assign when done debugging 
        angleRef = CalcAng(Odet,cntDet); 
        Dist = CalcDist(Odet, angleRef, cntDet); 
        Dadj = CalcDadj(Odet, cntDet, angleRef); 
        angleNormal = CalcNormAngle(Dadj, Dist); 

        // area 0 object 
        object0.distance = Dist; 
        object0.angle = angleNormal; 
        
    } 

    ////
    // For obstacles falling into region D1 
    ////
    cnt3 = 0; 
    cntDet = 0; 
    if (O1[0].dist != 0)
    { 
        for (cnt3 = 0; cnt3 <5; cnt3 ++)
        {
            if(O1[cnt3].dist != 0)
            {
                Odet[cnt3]= O1[cnt3]; // sorting obstacles from O1 into detection array 
                cntDet ++; 
            }
        }
        angleRef1 = CalcAng(Odet,cntDet); 
        Dist1 = CalcDist(Odet, angleRef1, cntDet); 
        Dadj1 = CalcDadj(Odet, cntDet, angleRef1); 
        angleNormal1 = CalcNormAngle(Dadj1, Dist1); 
        // area 1 object 
        if ((Dist1!=Dist) & (angleNormal != angleNormal1))
        {// only declaring new obstacle if it's distinct from pervios 
            object1.distance = Dist1; 
            object1.angle = angleNormal1; 

        }
        
    } 

    ////
    // For obstacles falling into region D2 
    ////
    cnt3 =0 ; 
    cntDet = 0; 
    if (O2[0].dist != 0)
    { 
        for (cnt3 = 0; cnt3 <5; cnt3 ++)
        {
            if(O2[cnt3].dist != 0)
            {
                Odet[cnt3]= O2[cnt3]; // sorting obstacles from O2 into detection array 
                cntDet ++; 
            }
        }
        angleRef2 = CalcAng(Odet,cntDet); 
        Dist2 = CalcDist(Odet, angleRef2, cntDet); 
        Dadj2 = CalcDadj(Odet, cntDet, angleRef2); 
        angleNormal2 = CalcNormAngle(Dadj2, Dist2); 

        // area 2 object 
        if ((Dist1!=Dist2) & (angleNormal2 != angleNormal1))
        {// only declaring new obstacle if it's distinct from pervios 
            object2.distance = Dist2; 
            object2.angle = angleNormal2; 
        } 
    } 


    ////
    // For obstacles falling into region D3
    ////
    cnt3 = 0;
    cntDet = 0;  
    if (O3[0].dist != 0)
    { 
        for (cnt3 = 0; cnt3 <5; cnt3 ++)
        {
            if(O3[cnt3].dist != 0)
            {
                Odet[cnt3]= O3[cnt3]; // sorting obstacles from O3 into detection array 
                cntDet ++; 
            }
        }
        angleRef3 = CalcAng(Odet,cntDet); 
        Dist3 = CalcDist(Odet, angleRef3, cntDet); 
        Dadj3 = CalcDadj(Odet, cntDet, angleRef3); 
        angleNormal3 = CalcNormAngle(Dadj3, Dist3); 

         // area 3 object 
        if ((Dist2!=Dist3) & (angleNormal2 != angleNormal3))
        {// only declaring new obstacle if it's distinct from pervios 
            object3.distance = Dist3; // changed end count from 1 to 3 bc that seemd logical IF THINGS FUCK UP CHANGE THIS BACK
            object3.angle = angleNormal3; 
        }
    } 
    // Array to hold detected obstacles 
    detected[0] = object0; 
    detected[1] = object1; 
    detected[2] = object2; 
    detected[3] = object3; 



   //
   //// Print Outputs of calculated data to terminal, for test/debug/validation purposes 
   //
   
  
    //
    //// section to send obstacle data to NAV MCU 
    // sending closest obstalce first 
    int scnt = 0; // send loop count. 
    
    for ( scnt = 0; scnt <=4; scnt++)
    {
        if(detected[scnt].distance != 0)
        {   // repalce cout with uart send  
            
            // UART SEND Obstacle[scnt].distance, Obstacle[scnt].angle to NAV MCU 
        }
    }
    
    return 0; 

}

int sideDetect(struct Reading S1, struct Reading S2)
{ 
// side obstacle detection
/* uses same principal as front detection 
modified to use only 2 sensors, 

important: 
sensor seperation is different

All distances in cm 
all angles in degrees ( unless otherwise noted) 

ASSUMES     S1 is towards front of mover and S2 is to rear 
*/ 

// struct to hold sensor readings, Dummy for now 
    
// sensor seperation. needs correct value when physically implemented /////////////////////////////////////////////////////////////////////
    float Sep = 25; 



    ////////////////////////////////////////////////////////////////////////////
    // dummy inputs 
    ////////////////////////////////////////////////////////////////////////////
    struct Reading R6; 
    R6.dist = 15.0; 
    R6.senNum = 1; 

    struct Reading R7; 
    
    R7.dist = 10.0; 
    R7.senNum = 2; 
    
    S1.dist = R6.dist; 
    S1.senNum = R6.senNum; 
    
    S2.dist = R7.dist; 
    S2.senNum = R7.senNum; 
            


    // boolean to determine if we are closing on any obstacle 
    // using 1 for closing (ie true) 0 for false 
    int closing = 1; 
    // minimum distance to obstalce (min sensor reading) 
    float minDist = 0.0; 
    // angle the obstacle makes with area object. either approaching or moving away from 
    float ang = 0.0; 


    
    float distDiff = 0.0; // difference in distance readings 
    // if we're closing 
    if (S1.dist < S2.dist)
    {   minDist = S1.dist; 
        distDiff = S2.dist - S1.dist; 
        closing = 1; 
    }
    // if we're moving away 
    else if (S1.dist > S2.dist)
    {
        minDist = S2.dist; 
        distDiff = S1.dist - S2.dist; 
        closing = 0; 
    }
    // if we are parrallel to the object 
    else 
    { 
        minDist = S1.dist; 
        ang = 0.0; 
    }
    // calculating anlge, using right triangle trig 
    ang = (180/3.141) * atan(distDiff/Sep);

    if (ang != minDist)
    { 
        ang = minDist; 
    }
    // output. Dummy for now, needs coordinated with JOSH 
    if (closing == 1)
    {
        // UTX cout << "CLOSING" << endl << " w angle: " << ang << " @ " << minDist << endl;  
    }
    else if (closing != 1) 
    { 
        // UTX cout << ang << endl << minDist << endl; 
    }

    return 0; 

}

float CalcAng (struct Reading OValid[5], int cntDet )
{
    // REFERANCE ANGLE FUNCTION//
    /* Given at least 2 reading objects this function computes
    the left most inside angle of the trianlge made between the 
    2 sensors and the object they are resolving 
    using the law of cosines 
    */ 

    // distance between the outermost sensors resolving the object 
    float Sep = fabs (OValid[0].senNum - OValid[cntDet-1].senNum)*SenSep;
    // Numerator for arccos argument 
    float arg1 =   ( pow(OValid[0].dist,2.0) - pow(OValid[cntDet-1].dist,2.0) - pow(Sep,2.0) ); 
    // Denominator for arccos argument 
    float arg2 = -2.0 * (OValid[cntDet-1].dist)*(Sep); 
    // finding angle with arccos and converting to degrees 
    float argcDeg = acos(arg1/arg2) * (180/3.141); 
    
    // returining angle in degrees 
    return argcDeg;   
}

float CalcDist(struct Reading OValid[5], float angle, int cntDet)
{
    /* function calcuates the perpnedicaulr distance from the mower 
    to the detected object given the distance from sensor cntDet 
    and the angle the line from S cntDet makes with the axel line of the mower 
    using rigth triangle trig
    */ 

    float dist = (OValid[cntDet-1].dist) * sin(angle*(3.141/180)); 
    return dist; 
}
float CalcDadj(struct Reading OValid[0], int cntDet, float angle)
{ 
    float Dadj = 0.0;
     // Adjacent side distance, distance from center sensor to point where line perpendicular to front of mower through objet hits  
    float Nseps = 0.0; // Number of integer multiple sensor seperations 
    float Fseps = 0.0; // Fraction of full sensor seperation for Dadj
    

    /* for cases where outer most resolving sensors sit on opposite sides of the center line 
    we need to take the distance from one sensor to the center, and then the distance 
    from the normal line to the sensor providing us the angle referance 
    and subtract the 2, giving us the distance from the normal line intersect to the center of the mower
    these are the 1st 2 cases. 

    When both sensors are on the same side of the mower, up to and including the center sensor 
    we need to add the 2 numbers instead to get the distance from normal line to center line
    (3rd case)

    Look at word doc and OneNote doccument for details.

    notice angle convention, objects to left have negative Dadj and hence negative normal angle 
    when reported out. set up in conditional blocks below.  
    */ 

   // Both sensors on either side of the center line 
    if ( (OValid[cntDet-1].senNum > 3) & (OValid[0].senNum < 3) )
    {
        // Full sensor seperations between center off array 
        // and point where normal line from obstacle lands 
        Nseps = (OValid[cntDet-1].senNum - 3) * SenSep; // distance from far sensor to center line 
        Fseps = cos((3.141/180)*angle)*OValid[cntDet-1].dist; // distance from normal line to reading sensor 
        if (OValid[cntDet-1].dist < OValid[0].dist)
        {
            Dadj = (Fseps-Nseps); 
        }
        else 
        { 
            Dadj = -(Fseps - Nseps); 
        }
    }
    // cross center calculation also, but the other way 
    // the way OValid gets filled this case should never happen during opperation
    else if ( (OValid[cntDet-1].senNum < 3) & (OValid[0].senNum > 3))
    {
        Nseps = (OValid[0].senNum - 3) * SenSep; // distance from far sensor to center 
        Fseps = cos((3.141/180)*angle)*OValid[cntDet-1].dist; // horizontal distance from normal line to sensor 
    }
    // Object detected by sensors on left side of cneter or center sensor 
    else if ( (OValid[cntDet-1].senNum <=3) & (OValid[0].senNum<=3) )
    {   // integer num of sensor seperations if both sensors are on the same side of the center, 
        // or include center sensors 
        Nseps = (fabs(OValid[cntDet-1].senNum - 3)) * SenSep;
        Fseps = cos((3.141/180)*angle) * OValid[cntDet-1].dist;  
        Dadj = -(Nseps +Fseps); 
    }
    // obstacle resolved by sensors on right side of center, or center 
    else if ( (OValid[cntDet-1].senNum >=3) & (OValid[0].senNum >=3) )
    {
        // Distance from left most sensor to center  
        Nseps = fabs (OValid[0].senNum - 3)* SenSep;  
        // Sensor seperations between resolving sensors minus adjacent side to referance angle (from rigth most senstor)
        Fseps =  (fabs(OValid[cntDet-1].senNum - OValid[0].senNum)*SenSep) - (cos((3.141/180)*angle)*OValid[cntDet-1].dist);   
        Dadj = Nseps +Fseps;
    }
    // passted this line back into conditional block, changed from + to minus in first and 2nd Dadj = Nseps +Fseps; 

    return Dadj; // length of line adjacent for right triangle trig, 
    // distance from normal line of object to mower to center of mower ( as defined as central sensor. SenNum 3)
}

float CalcNormAngle(float Dadj, float dist)
{ 
    /* calculates the angle a line from the center of the mower to the object 
    makes with the front of the mower
    in degrees (hopefully)
    //90 deg indicates object straight ahead
    */
    float Theta = atan(dist/Dadj) * (180/3.141); 
    return Theta; 
}
/* *****************************************************************************
 End of File
 */
