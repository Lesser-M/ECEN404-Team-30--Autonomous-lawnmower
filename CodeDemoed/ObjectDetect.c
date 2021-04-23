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
        int TmrLoopCheck = 0; // reads timer value on current itteration of loop 
        int TmrLoopCheckL = 0; // holds timer value of last loop itteration 
        int TMRLoops = 0; // counts the number of timer loops 
        bool stuck = false; // tells us if we are stuck in an infinite loop 
        /* the timer period is 40ms, all sensors should return echo pulse in less 
         * than 40ms. But since the timer may be at any value when we start we may
         * loop as much as once, and still be vaild. 
         * IF we loop more then once, than we are stuck. 
         * by comparing this itterations timer value to lasts we can see if the 
         * timer has reset. if it has reset, we incriment TMRLoops. If TMR loops >= 2 
         * we have not received a valid return from the sensors and break the loop 
         * by using the stuck bool 
         * If sensors fail we will not get the signals we are looking for below, 
         * and are left in an infinite loop
         */
        while ((!end) & (!stuck))
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
            // checks for infinite loops. see explanation above while loop 
            TmrLoopCheck = TMR3_CounterGet(); 
            if (TmrLoopCheck > TmrLoopCheckL)
            {
                TmrLoopCheckL = TmrLoopCheck; 
            }
            else { 
                TMRLoops = TMRLoops + 1; 
                TmrLoopCheckL = 0; 
            }
            if (TMRLoops >= 2)
            {
                stuck = true;
                
            }
            if(SenStart == 5) // we have start readings from all, can begin looking for ends 
            {  
                TMRLoops = 0; 
                TmrLoopCheckL = 0; 
                stuck = false; 
                while ((!end) & (!stuck))
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
                    // including TMR loop check from initial while loop here also
                    // incase we get stuck looking for the echo 
                    TmrLoopCheck = TMR3_CounterGet(); 
                    if (TmrLoopCheck > TmrLoopCheckL)
                    {
                        TmrLoopCheckL = TmrLoopCheck; 
                    }
                    else { 
                        TMRLoops = TMRLoops + 1; 
                        TmrLoopCheckL = 0; 
                    }
                    if (TMRLoops >= 3)  // made this 3 to account for propagation delays 
                    {                   // don't think that's the issue, but don't want to chase it if it is
                        stuck = true;
                
                    }    
                }
            }
        }
        // SEE S1 BLOCK FOR EXPLANATION 
        // also note: using if, elseif block, to ignore case where start and finish
        // ticks are equal. TMR period is longer than max sensor read period, 
        // hence Finish == Start can only be caused by error. 
        // can introcude check here it this starts happending for some reasson 
        // timer ticks for ToF from sensor1 
        int dif_S1;
        if (S1_Finish > S1_Start)
        {   // normal case
            dif_S1 = S1_Finish-S1_Start; // ToF for sensor 1 relative to timmer 3 
        }
        else if (S1_Finish < S1_Start)
        {   // incase timer rolls over 
            // 1562500 is max tmr value, subtracting start value from that, and 
            // adding finish value, given start>finish 
            // gives number of timer ticks 
            dif_S1 = (1562500-S1_Start)+S1_Finish; 
        }
        // S2 TMR tick 
        int dif_S2; 
        if (S2_Finish > S2_Start)
        {
            dif_S2 = S2_Finish-S2_Start; 
        }
        else if (S2_Finish < S2_Start)
        {
            dif_S2 = (1562500 - S2_Start) + S2_Finish; 
        }
        // S3 TMR Ticks 
        int dif_S3; 
        if (S3_Finish > S3_Start)
        {
            dif_S3 = S3_Finish-S3_Start; 
        }
        else if (S3_Finish < S3_Start)
        {
            dif_S3 = (1562500 - S3_Start) + S3_Finish; 
        }
        // S4 TMR Ticks
        int dif_S4; 
        if (S4_Finish > S4_Start)
        {
            dif_S4 = S4_Finish-S4_Start; 
        }
        else if (S4_Finish < S4_Start)
        {
            dif_S4 = (1562500 - S4_Start) + S4_Finish; 
        }
        // S5 TMR Ticks 
        int dif_S5; 
        if (S5_Finish > S5_Start)
        {
            dif_S5 = S5_Finish-S5_Start; 
        }
        else if (S5_Finish < S5_Start)
        {
            dif_S5 = (1562500 - S5_Start) + S5_Finish; 
        }
            
        
        
        // calculating distance from timer ticks, see function description for formula  
        float distance1 = (dif_S1)*0.01088; // ToF in ms * Vsound in M/s * (1/1000)[s/ms]*(100[cm/M])
        float distance2 = 169.0 + (0.0000001* dif_S2); //dif_S2 * 0.01088;  // sensor was reporting wrong, eliminated  
        //float distance3 = dif_S3 *0.01088; 
        float distance4 = dif_S4 * 0.01088; 
        float distance5 = dif_S5 * 0.01088;
        float distance3 = 169.0 + (dif_S3*0.0000001); // sensor was reporting wrong, eliminated  
        // we include the stuck variable from the sensor reading loops above 
        // in the initial validation here 
        // if the loop got stuck at any time, we want discard all meassuremnts 
        
        stuck = false; // was used as timer error check, but sensor issues are causing error
        // by setting to false we just use the lowest sensor reading, 
        // without validating 
        if ((distance1 <=400)&(!stuck))
        {
            R1.dist = distance1; 
        }
        else 
        {
            R1.dist = 0.0; 
        }
        if ((distance2 <=400)&(!stuck))
        {
            R2.dist = distance2; 
        }
        else 
        {
            R2.dist = 0.0; 
        }
        if ((distance3 <=400) &(!stuck))
        {
            R3.dist = distance3; 
        }
        else 
        {
            R3.dist = 0.0; 
        }
        if ((distance4 <=400)&(!stuck))
        {
            R4.dist = distance4; 
        }
        else 
        {
            R4.dist = 0.0; 
        }
        if ((distance5 <=400)&(!stuck))
        {
            R5.dist = distance5; 
        }
        else 
        {
            R5.dist = 0.0; 
        }
        // for debug only, let's us see when the sensor real loop(s) freeze 
        /*
        if(stuck)
        {
            UART5_Write("STUCK\n\r",7);
            DelayMs(500); 
            R1.dist = 400.0;                    // had some crashes that printed "STUCK" right before crash  
            R2.dist = 400.0;                    // incase this is from caused by 0 distance inputs 
            R3.dist = 400.0;                    // I'm writing 400 to all values here 
            R4.dist = 400.0; 
            R5.dist = 400.0;            
                    
        }
         */ 
        R1.senNum = 1; 
        R2.senNum = 2; 
        R3.senNum = 3; 
        R4.senNum = 4; 
        R5.senNum = 5; 
        //float distance = distance1+distance2+distance3+distance4+distance5;
        detect(R1,R2,R3,R4,R5);
        return 0; 
        /* Sensors are not returning valid data. 
         Suspect Hardware issue
         Worked during first array test. But once array was installed on the mower
         distance data is no longer valid, and timer loop runs more than twice */
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
    
// Trying something simpler 
    /* The code below was written to simplify obstacle detection 
     due to inaccurate sensors and unpredictability of obstacles, 
     * the previous code calculating angles and using multiple resolution zones 
     * proved to complex to yield consistently accurate resutls 
     * The code below works in preliminary test, but might need bound for 
     * straight ahead objest corrected 
     *  theory of opperation: 
     * the code receives the 5 readings from the read front array function, 
     * sorts them into an array for further processing. 
     * Next it discards any invalid readings, that is distances outside of 
     * (2, 400) cm range of the sensors. 
     * In the valid readings array it next finds the minimum distance. 
     * This gets reported as minimum distance and sent to NMCU. 
     * 
     * if the minimum distance is from sensor 3, the middle sensor 
     * it finds the difference between the 2 sensors on either side of it. 
     * it this distance is small enough ( currently 2cm, may need revised) 
     * it determines the object is straight ahead. 
     * Otherwise, if the sensor to the left of center reports lower distance than 
     * the sensor to the right, the obstacle is left of center, otherwise right 
     * of center. 
     * 
     * IF the lowest distance is not reported by the middle sensor, than the side
     * on which the sensor with the lowest distance is on, is also the side 
     * on which the obstacle is.
     * 
     * Sides in this context refer to an observer looking in the (forward) direction 
     * to mower drives 
     */
    
    struct Reading Read[5];                 // holds read values, for easeir proceesing 
    Read[0] = F1;                           // Sorting received readings into Read array  
    Read[1] = F2; 
    Read[2] = F3; 
    Read[3] = F4; 
    Read[4] = F5; 
    struct Reading Valids[5];               // holds valid (2<dist<=400) readings 
    int cn1;                                // loop count to sort Read into valid  
    int cn2;                                // Loop count to track elements in valid, gets incrimented only when we add
    cn2 = 0;                                // an element to valid, and starts at o  
    
    int side;                               // tells us which side the object is on 
                                            // looking forward from mowers perspective 
                                            // 0 = right of center 
                                            // 1 = left of center 
                                            // 2 = Straight ahead 
    
    // sorts all readings with valid distanced into valid array
    // sensors go from left to right 
    for (cn1 = 0; cn1 <5; cn1++)
    {
        if((Read[cn1].dist > 2) & (Read[cn1].dist <=400))   // checking if the reading is valid
        {
            Valids[cn2] = Read[cn1];                        // and if so adding it to valid array 
            cn2++;                                          // and keeping track of the number of elements in it 
        }   
    }
    
    struct Reading min;                     // struct to hold reading with shortest distance  
    min.dist = 400.0;                       // setting distance to max, s0 that we can sort 
    int cn3;                                // count to sort through valid array for minimum distance 
    int cn4;                                // tracks where in valid array the minim distance reading is 
    cn4 = 0;                                // starts at 0 
    for (cn3 = 0; cn3 < cn2; cn3++)
    {
        if (Valids[cn3].dist < min.dist)    // finding next (new) min 
        {
            min = Valids[cn3];              // assigning it to min reading 
            
            cn4++;                          // and keeping track of where it is 
        }   
    }
    // finding what side the obstacle is on 
    if (min.senNum == 3)                    // if sensor 3 reports lowest distance the object might be straight ahead 
    {   
        if ( abs (Valids[cn4-1].dist - Valids[cn4+1].dist) < 2) 
        {                                   // if sensors either side to center are within 2 cm, we conclude object is straight ahead
            // straight
            side = 2;
        }
        else if (Valids[cn4-1].dist > Valids[cn4+1].dist)
        {                                   // if it's closest to center, but closer to the right than left, we say it's on the right 
            // right 
            side = 0; 
            
        }
        else 
        {                                   // and if it's closer to the left than right, we say it's on the left 
            // left 
            side = 1; 
        }
         
    }                                       // otherwise, if the minimum reading is on the left side of the array, so is the object 
    else if(min.senNum < 3)
    {
        // left 
        side = 1; 
    }
    else if(min.senNum > 3)
    {                                       //and finally the left  
        // right 
        side = 0; 
    }
//    else
//    {
//        // probelm 
//    }
//    if (cn4 == 0)
//    {
//        // object is on far left side 
//    }
//    else if (cn4 == cn3)
//    {
//        // object on far right side 
//    }
//    else 
//    {
//        if(Valids[cn4-1].dist < Valids[cn4])
//    }
    // add error bounds 
    // abs(V0-Vcn) < 2? 10?
//    if (abs(Valids[0].dist - Valids[cn2].dist) > 2)
//    {
//        if (Valids[0].dist < Valids[cn2].dist)
//        {
//            // object is left of center 
//            side = 1; 
//             
//        }
//        else if (Valids[0].dist > Valids[cn2].dist)
//        {
//        // object is right of center 
//           
//            side = 0;  
//        }       
//        
//        // object is closer to left side than right, meaning it is on the right 
//        // with some error bound 
//    }
//    else 
//    {
//        // object is straight ahead 
//        
//        side = 2; 
//    }
    
    // finds smallest distance reported 
//    struct Reading min;  
//    min.dist = 400.0;
//    int cn3; 
//    for (cn3 = 0; cn3 < cn2; cn3++)
//    {
//        if (Valids[cn3].dist < min)
//        {
//            min.dist = Valids[cn3].dist; 
//        }
//        
//    }
    // converting numbers to strings so we can send via UART 
    char dist[3];                       // distance  
    char sideC[2];                      // what side we are on ( R, L, S)
    memset(sideC , ' ', 2);             // initializing side to all spaces, so we don't send garbage 
    if(side == 2)                       // if side == 2 object is straight ahead, see above 
    {
        memset(sideC, 'S', 1); 
    }
    else if (side == 1)                 // and if side == 1 it's to the left 
    {
        memset(sideC, 'L', 1); 
    }
    else if (side == 0)                 // and finally 0 means to the right 
    {
        memset(sideC, 'R', 1); 
    }
                                        // Note that if we haven't found the side the object is on, that field stays blank 
    memset(dist,' ', 3);                // initializing dist to all spaces too,  
                                        // we need 3 postitions as our distnace goes up to 400 
                                        // but we may only need 2 of those (dist<100) so this memset is importnat 
                                        // to prevent us from sending unwanted charachters 
    itoa(dist, min.dist, 10);           // converting minimum distance to a string  
     
    // Outputting results, with (adjustable) delays 
    // may want to convert this into one string 
    char output[9]; 
    memset(output, ' ', 9);
    
    if (min.dist <40)        // changed min to R1
    {
        // outputting only when object within 80cm, 
        // as per NAV-MCU config 
        GPIO_RF2_Set(); 
        // could set to toggle instead of set, 
        // but i fear if we do that it'll toggle each cycle, causing the 
        // ESPs ISR to tigger each cycle an obstacle was detected, 
        // even if it is the same one. 
        // this way it should set high once it sees the obstacle, stay high while it sees
        // the obstacle, and reset low once it no longer sees it. 
        // causing one ISR call on the esp per "seen" obstacle 
//        UART5_Write(dist,2);
//        DelayMs(100); 
//        UART5_Write(",",1); 
//        DelayMs(10); 
//        UART5_Write(sideC,1);
//        DelayMs(10); 
        
    }
    else 
    {
        // no obstacle within 80 cm 
        GPIO_RF2_Clear(); 
    }
    // sprintf(output,"%s %s %s", dist, ",", sideC);
    // fails to carrige return, otherwise good 
    /*
    UART5_Write(dist, 3); 
    DelayMs(100);     //UART5_Write(dist,3);
    UART5_Write(",",1); 
    DelayMs(100); 
    UART5_Write(sideC,2); 
    DelayMs(100); 
     
    UART5_Write("\n\r",2); // doing cr lf here now S
    DelayMs(10); 
     */

    
    
    
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////BEGIN DECLARATION OF VARIBLES /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    //
//    ////
//    ////// struct arrays holding readings at various stages of the sorting process 
//    ////
//    //
//    struct Reading O[5] = {F1,F2,F3,F4,F5}; // sensor readings
//    //// structs for sorted readings 
//    // by distance: 
//    struct Reading O0[5] = {}; // Readings between D1 and Do 
//    struct Reading O1[5] = {}; // Readings greater than D1 
//    struct Reading O2[5] = {}; // Readings greater than D2 
//    struct Reading O3[5] = {}; // Readings greater then D3
//    
//    // for processing sensors w. readings in the same region
//    struct Reading Odet[5] = {}; // Readings resolving the same obstacle 
//
//    // array holding detected obstacles, with Normal angle and distance 
//    struct Obstacle detected[5] = {}; 
//
//   //
//   ////
//   ////// Internal count and tracking varibles used in sorting loops and validation procedures 
//   ////
//   //
//   
//    int cnt = 0; // counts for sorting loops 
//    int cnt2 = 0; // count for sorting loops 
//
//    int cnt3 = 0; // loop count to move elements from area Arrays into detected object array 
//    int cntDet = 0; // count of sensors with valid readings in the given detection range 
//  
//    //
//    ////
//    ////// Distances defining different boundary regions, where point obstacles can be resolved with different numbers of sensors 
//    ////// not including error bounds, those are added in the applicable sorting loops themselves 
//    ////
//    //
//    float D0 = 3.0; // minimum distance sensor can resolve 32.54; // minimum distance to resolve object w 2 sensors 
//    float D1 = 65.09; // minimum distance to resolve object w 3 sensors
//    float D2 = 97.64; // minimum distance to resolve object w 4 sensors 
//    float D3 = 130.19; // minimum distance to resolve object w 5 sensors
// 
//   //
//   ////
//   ////// Variables to hold results or intermediate values needed to calcualte results, to be passed to calcualtion functions 
//   ////
//   //
//    float Dadj; // Adjacent distance, intermediate step to result 
//    float Dist; // distance from mower to object, result parameter 
//    float angleRef; // angle between line from refence sensor to object and front of mower, intermediate value 
//    float angleNormal; // angle between front of mower and line from cneter of mower to object
// 
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////END DECLARATION OF VARIBLES /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  
//    ////
//    ////
//    // Loops to sort readings according to boundary region they fall into 
//    ////
//    ////
//    // if we start missing detections check these region bounds  
//    
//    for (cnt = 0; cnt <5; cnt++)
//    { // possible objects further than D3 
//        if (O[cnt].dist >= D3)
//       {
//           O3[cnt2] = O[cnt];
//           cnt2++; 
//       } 
//    }
//    cnt = 0;
//    cnt2 = 0;  
//    for (cnt = 0; cnt <5; cnt++)
//    { // possible objects between D3 and D2 including some error bound overlap 
//        if ( (O[cnt].dist >= D2) & (O[cnt].dist <= (D3 + maxdiff)))
//       {
//           O2[cnt2] = O[cnt];
//           cnt2++; 
//       } 
//    }
//    cnt = 0; 
//    cnt2= 0; 
//    for (cnt = 0; cnt <5; cnt++)
//    { // possible objects between D2 and D1 
//        if ((O[cnt].dist >= D1) & (O[cnt].dist <= (D2 + maxdiff) ))
//       {
//           O1[cnt2] = O[cnt];
//           cnt2++; 
//       } 
//    }
//    cnt = 0; 
//    cnt2 = 0; 
//    for (cnt = 0; cnt <5; cnt++)
//    { // possible objects between D0 and D1  
//        if ( (O[cnt].dist >= D0) & ( O[cnt].dist <= (D1 + maxdiff)))
//       {
//           O0[cnt2] = O[cnt];
//           cnt2++; 
//       } 
//    }
//
//
//
//    ////
//    ////
//    // Grouping readings for same object from boundary regions, beginning with closest 
//    ////
//    ////
//    /////////////////////////////////////////////////////////////////////////////////////////////////////////
//    /* old
//    Repeate this process for the other detection zones. 
//    Pass Odet array to angle and distance solve functions outlined below 
//
//    remaining issues, what when we have multiple sensor readingfs in a zone refering to different objects? 
//    What about area objects? 
//    */
//   ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//    // Variables to track up to 3 different obstacles 
//    float angleRef1 = 0.0; 
//    float Dist1 = 0.0; 
//    float Dadj1 = 0.0; 
//    float angleNormal1 = 0.0;
//    // obstacle 2 
//    float angleRef2 = 0.0; 
//    float Dist2 = 0.0; 
//    float Dadj2 = 0.0; 
//    float angleNormal2 = 0.0;
//    // obstacle 3 
//    float angleRef3 = 0.0; 
//    float Dist3 = 0.0; 
//    float Dadj3 = 0.0; 
//    float angleNormal3 = 0.0;
//
//    // detected obstacles in different areas
//    // any number of sensor readings in an area will be treated as 
//    // an area obsatcle 
//
//    struct Obstacle object0;
//    struct Obstacle object1;
//    struct Obstacle object2;
//    struct Obstacle object3;
//    // initialzing distance so we can find empties later 
//    object0.distance = 0; 
//    object1.distance = 0; 
//    object2.distance = 0; 
//    object3.distance = 0; 
//
//
//    
//    ////// 
//    ////// NOTE FOR ALL Odet Arrays, senseros in array are ordered left to right when looking in dirve direction 
//    //////
//
//    ////
//    // For obstacles falling into region D0 
//    ////
//    if (O0[0].dist != 0)
//    { 
//        for (cnt3 = 0; cnt3 <5; cnt3 ++)
//        {
//            if(O0[cnt3].dist != 0)
//            {
//                Odet[cnt3]= O0[cnt3]; // sorting obstacles from O0 into detection array 
//                cntDet ++; 
//            }
//        }
//        // change these block to directly assign when done debugging 
//        angleRef = CalcAng(Odet,cntDet); 
//        Dist = CalcDist(Odet, angleRef, cntDet); 
//        Dadj = CalcDadj(Odet, cntDet, angleRef); 
//        angleNormal = CalcNormAngle(Dadj, Dist); 
//
//        // area 0 object 
//        // Validation, if distance is not in (2,400) cm, or |angle| > 90 deg, we have an error 
//       // Validate angle first, then distance, that way if dist is valid, so is angle 
//        
//        if (abs(angleNormal) <= 90)
//        {
//            object0.angle = angleNormal;
//            if ((Dist > 2) && (Dist < 400))
//            {
//                object0.distance = Dist; 
//            // initialized to 0 
//            }
//        }
//         
//        
//    } 
//
//    ////
//    // For obstacles falling into region D1 
//    ////
//    cnt3 = 0; 
//    cntDet = 0; 
//    if (O1[0].dist != 0)
//    { 
//        for (cnt3 = 0; cnt3 <5; cnt3 ++)
//        {
//            if(O1[cnt3].dist != 0)
//            {
//                Odet[cnt3]= O1[cnt3]; // sorting obstacles from O1 into detection array 
//                cntDet ++; 
//            }
//        }
//        angleRef1 = CalcAng(Odet,cntDet); 
//        Dist1 = CalcDist(Odet, angleRef1, cntDet); 
//        Dadj1 = CalcDadj(Odet, cntDet, angleRef1); 
//        angleNormal1 = CalcNormAngle(Dadj1, Dist1); 
//        // area 1 object 
//        if ((Dist1!=Dist) & (angleNormal != angleNormal1))
//        {// only declaring new obstacle if it's distinct from previous 
//            // Validation, if distance is not in (2,400) cm, or |angle| > 90 deg, we have an error 
//            if ( (Dist1 > 2) && (Dist1 <=400))
//            {
//                object1.distance = Dist1;
//            }
//            if (abs(angleNormal1) <= 90)
//            {
//                object1.angle = angleNormal1;
//                if ( (Dist1 > 2) && (Dist1 <=400))
//                {
//                    object1.distance = Dist1;
//                }
//            }
//
//        }
//        
//    } 
//
//    ////
//    // For obstacles falling into region D2 
//    ////
//    cnt3 =0 ; 
//    cntDet = 0; 
//    if (O2[0].dist != 0)
//    { 
//        for (cnt3 = 0; cnt3 <5; cnt3 ++)
//        {
//            if(O2[cnt3].dist != 0)
//            {
//                Odet[cnt3]= O2[cnt3]; // sorting obstacles from O2 into detection array 
//                cntDet ++; 
//            }
//        }
//        angleRef2 = CalcAng(Odet,cntDet); 
//        Dist2 = CalcDist(Odet, angleRef2, cntDet); 
//        Dadj2 = CalcDadj(Odet, cntDet, angleRef2); 
//        angleNormal2 = CalcNormAngle(Dadj2, Dist2); 
//
//        // area 2 object 
//        if ((Dist1!=Dist2) & (angleNormal2 != angleNormal1))
//        {// only declaring new obstacle if it's distinct from pervios 
//            // Validation, if distance is not in (2,400) cm, or |angle| > 90 deg, we have an error 
//            
//            if (abs(angleNormal2) <= 90)
//            {
//                object2.angle = angleNormal2; 
//                if ( (Dist2 > 2) && (Dist2 <=400))
//                {
//                    object2.distance = Dist2;
//                }
//            }
//             
//        } 
//    } 
//
//
//    ////
//    // For obstacles falling into region D3
//    ////
//    cnt3 = 0;
//    cntDet = 0;  
//    if (O3[0].dist != 0)
//    { 
//        for (cnt3 = 0; cnt3 <5; cnt3 ++)
//        {
//            if(O3[cnt3].dist != 0)
//            {
//                Odet[cnt3]= O3[cnt3]; // sorting obstacles from O3 into detection array 
//                cntDet ++; 
//            }
//        }
//        angleRef3 = CalcAng(Odet,cntDet); 
//        Dist3 = CalcDist(Odet, angleRef3, cntDet); 
//        Dadj3 = CalcDadj(Odet, cntDet, angleRef3); 
//        angleNormal3 = CalcNormAngle(Dadj3, Dist3); 
//
//         // area 3 object 
//        if ((Dist2!=Dist3) & (angleNormal2 != angleNormal3))
//        {// only declaring new obstacle if it's distinct from pervios 
//            // Validation, if distance is not in (2,400) cm, or |angle| > 90 deg, we have an error 
//            
//            if (abs(angleNormal3) <= 90)
//            {
//                object3.angle = angleNormal3; 
//                if ( (Dist3 > 2) && (Dist3 <=400))
//                {
//                    object3.distance = Dist3;
//                }
//            }
//            
//        }
//    } 
//    // Array to hold detected obstacles 
//    // Only load obstacesl into detection array if they have a valid distance
//    // angle is validated before distance, so valid distance implies valid angle 
//    if (object0.distance != 0)
//    {
//        detected[0] = object0; 
//    }
//    if (object1.distance != 0)
//    {
//        detected[1] = object1; 
//    }      
//    if (object2.distance != 0)
//    {
//        detected[2] = object2; 
//    }
//    if (object3.distance != 0)
//    {
//        detected[3] = object3; 
//    }
////    detected[1] = object1; 
////    detected[2] = object2; 
////    detected[3] = object3; 
//    // strings for uart output, filled w white spaces 
//    char Dout[4]; 
//    memset(Dout, ' ', 4);
//    char Aout[3]; 
//    memset(Aout, ' ', 3); 
//    //int DistOut; 
//    //int AngOut; 
//
//
//
//   //
//   //// Print Outputs of calculated data to terminal, for test/debug/validation purposes 
//   //
//   
//  
//    //
//    //// section to send obstacle data to NAV MCU 
//    // sending closest obstalce first 
//    int scnt = 0; // send loop count.
//    
//    for (scnt = 0; scnt <4; scnt++)
//    {
//        if(detected[scnt].distance != 0)
//        {   // repalce cout with uart send  distance
//        // reverted back to loop, incase we have multiple obstacles, 
//        // let's see how it works. 
//        
//            itoa(Dout, detected[scnt].distance, 10); 
//            itoa(Aout, detected[scnt].angle, 10); 
//            UART5_Write("Objects: ", 9);
//            UART5_Write(Dout , 4);
//            DelayMs(100);
//            UART5_Write(" , ", 3); 
//            DelayMs(500);
//            UART5_Write(Aout, 3);  
//            DelayMs(500); 
//            UART5_Write("\n\r", 2); 
//        }
//    }
//    
    return 0; 

}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
int sideDetect(struct Reading S1, struct Reading S2)
{ 
// side obstacle detection
/* uses same principal as front detection 
modified to use only 2 sensors, 

important: 
sensor seperation is different

All distances in cm 
all angles in degrees ( unless otherwise noted) 

ASSUMES     Side1 is towards front of mover and Side2 is to rear 
 * Side 1 is on RD13 
 * Side 2 is on RJ1 
 * ////// THESE ASSIGMNETS WERE NOT YET TESTED DUE TO POWER OUTAGE 
 * Function cleans and builds but is not validated beyond that due to power outage 
 * Needs additional comentes for clarity, Defer process to later when power comes back 
*/ 

// struct to hold sensor readings, Dummy for now 
    bool end = false; // bool to check reading status 
    // Bools to check if we have start /end values for side sensor 1 
    bool Side1S = false; 
    bool Side1E = false;
    // tracks start and end timer values for side sensor 1 
    int Side1_Start = 0; 
    int Side1_End = 0; 
    // Bools to check if we have start /end values for side sensor 2
    bool Side2S = false; 
    bool Side2E = false; 
    // tracks start and end timer values for Side sensor 2 
    int Side2_Start = 0; 
    int Side2_End = 0; 
    
    int SideStarts = 0; // tracks num of side sensor that have start reading 
    int SideEnds = 0; // tracks num of side sensor that have end reading 
    
    // Pin 113 RD13 for front side sensor 
    // Pin 115 RJ1 for read side sensor 
    /******************************************************************************/
    // NEEDS IT'S OWN TRIGGER 
    /******************************************************************************/
    GPIO_RD13_InputEnable(); 
    GPIO_RJ1_InputEnable(); 

    
   
    
         
    
    GPIO_RB14_OutputEnable(); 
    // Triggers side array 
    GPIO_RB14_Set(); 
    DelayUs(10); 
    GPIO_RB14_Clear(); 
    // copied f. front array function 
    int TmrLoopCheck = 0; // reads timer value on current itteration of loop 
    int TmrLoopCheckL = 0; // holds timer value of last loop itteration 
    int TMRLoops = 0; // counts the number of timer loops 
    bool stuck = false; // tells us if we are stuck in an infinite loop 
        /* the timer period is 40ms, all sensors should return echo pulse in less 
         * than 40ms. But since the timer may be at any value when we start we may
         * loop as much as once, and still be vaild. 
         * IF we loop more then once, than we are stuck. 
         * by comparing this itterations timer value to lasts we can see if the 
         * timer has reset. if it has reset, we incriment TMRLoops. If TMR loops >= 2 
         * we have not received a valid return from the sensors and break the loop 
         * by using the stuck bool 
         * If sensors fail we will not get the signals we are looking for below, 
         * and are left in an infinite loop
         */
    // detail to be added. For now see front array read function 
    while ((!end)&(!stuck)) // looking for start readings 
    {
        if ((GPIO_RD13_Get() == 1)&(!Side1S))
        {   // Side front  
            Side1_Start = TMR3_CounterGet(); 
            SideStarts ++; 
            Side1S = true;
        } 
        if ((GPIO_RJ1_Get() == 1)&(!Side2S))
        {   //Side read 
            Side2_Start = TMR3_CounterGet(); 
            SideStarts++; 
            Side2S = true; 
        } 
        TmrLoopCheck = TMR3_CounterGet(); 
        if (TmrLoopCheck > TmrLoopCheckL)
        {
            TmrLoopCheckL = TmrLoopCheck; 
        }
        else if (TmrLoopCheck < TmrLoopCheckL)
        {
            TMRLoops = TMRLoops+1; 
            TmrLoopCheckL = 0; 
            
        }
        if (TMRLoops >=2)
        {
            stuck = true; 
        }
        if(SideStarts == 2) // we have start readings from all, can begin looking for ends 
        {
            TmrLoopCheckL = 0; 
            while ((!end)&(!stuck))
            {
                if ((GPIO_RD13_Get() == 0)&(!Side1E)) 
                {   //Side front 
                    Side1_End= TMR3_CounterGet(); 
                    SideEnds++; 
                    Side1E = true; 
                }
                if ((GPIO_RJ1_Get() == 0)&(!Side2E))
                {   //Side read 
                    Side2_End = TMR3_CounterGet(); 
                    SideEnds ++; 
                    Side2E = true; 

                }
                if (SideEnds == 2)
                {
                    end = true; 
                }
                TmrLoopCheck = TMR3_CounterGet(); 
                if (TmrLoopCheck > TmrLoopCheckL)
                {
                    TmrLoopCheckL = TmrLoopCheck; 
                }
                    else if (TmrLoopCheck < TmrLoopCheckL)
                {
                    TMRLoops = TMRLoops+1; 
                    TmrLoopCheckL = 0; 
            
                }
                if (TMRLoops >=2)
                {
                    stuck = true; 
                }
            }
        }
    }
    // tracking timer difference for side sensors 
    // validating incase TMR loop 
    int Side1diff = 0; 
    int Side2diff = 0; 
    
    if (Side1_Start < Side1_End)
    {
        Side1diff = Side1_End - Side1_Start; 
    }
    else 
    {
        Side1diff = (1562500 - Side1_Start)+Side1_End; 
    }
    if (Side2_Start < Side2_End)
    {
        Side2diff = Side2_End - Side2_Start; 
    }
    else 
    {
        Side2diff = (1562500 - Side2_Start)+Side2_End; 
    }
    
    struct Reading Side1; 
    struct Reading Side2; 
    
    float Side1Dist; 
    float Side2Dist; 
    Side1Dist = Side1diff * 0.01088;
    Side2Dist = Side2diff * 0.01088; 
    if (Side1Dist < 400)
    {
        Side1.dist = Side1Dist; 
    }
    else
    {
        Side1.dist = 0.0; 
    }
    if (Side2Dist < 400)
    {
        Side2.dist = Side2Dist; 
    }
    else 
    {
        Side2.dist = 0.0; 
    }
    Side1.senNum = 1; 
    Side2.senNum = 2; 
    
    
    
// sensor seperation. needs correct value when physically implemented /////////////////////////////////////////////////////////////////////
    float Sep = 23.8; // on unit



    
            


    // boolean to determine if we are closing on any obstacle 
    // using 1 for closing (ie true) 0 for false 
    //int closing = 1; 
    // minimum distance to obstalce (min sensor reading) 
    float minDist = 0.0; 
    // angle the obstacle makes with area object. either approaching or moving away from 
    float ang = 0.0; 


    
    float distDiff = 0.0; // difference in distance readings 
    // if we're closing 
    if (Side1.dist < Side2.dist)
    {   minDist = Side1.dist; 
        distDiff = Side2.dist - Side1.dist; 
        //closing = 1; 
    }
    // if we're moving away 
    else if (Side1.dist > Side2.dist)
    {
        minDist = Side2.dist; 
        distDiff = Side1.dist - Side2.dist; 
        //closing = 0; 
    }
    // if we are parrallel to the object 
    else 
    { 
        minDist = Side1.dist; 
        ang = 0.0; 
    }
    // calculating anlge, using right triangle trig 
    ang = (180/3.141) * atan(distDiff/Sep);
    // Converting angle and distance to strings for sending 
    char sideang[2]; 
    memset(sideang, ' ', 2); 
    char sidedist[3]; 
    memset(sidedist, ' ', 3); 
    itoa(sideang, ang, 10); 
    itoa(sidedist, minDist, 10); 
    ////////////////////////////////////////////////////////////////////////////
    // NOTE: ANGLE IS POSITIVE EITHER WAY (closing or not closing) , COORDINATE WITH JOSH FOR FORMATING 
    ////////////////////////////////////////////////////////////////////////////
    if (minDist < 50)
    {
        GPIO_RF2_Set(); 
    }
    else
    {
        GPIO_RF2_Clear(); 
    }
    /*
    UART5_Write("min dist: ", 10);
    DelayMs(50);
    UART5_Write(sidedist, 3); 
    DelayMs(50); 
    UART5_Write(" ", 1); 
    DelayMs(50); 
    UART5_Write("Angle: ", 7);
    DelayMs(50);
    UART5_Write(sideang, 2); 
    DelayMs(50);
    UART5_Write("\n\r",2); 
    
    if (closing == 1)
    {
         
        // set obstacle flag ( pin RB7 high) )
    }
    else if (closing != 1) 
    { 
        // else set obstacle flag ( pin RB7 low) )
    }
    */ 

    return 0; 

}
/******************************************************************************/
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
/******************************************************************************/
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
/******************************************************************************/
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
/******************************************************************************/
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
