/*******************************************************************************
  Main Source File
 * so meine freunde
  Company:
    Max Lesser For ECEN capstone F2020- S2021

  File Name:
    main.c
    depends on sensor_read.c /h 
               ObstacleDetect.c /h 
               device headers and definitins  

  Summary:
    This file contains the "main" function for a project.

  Description:
  
  SUMMARY OF OPPERATION AT BOTTOM OF MAIN 
  
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "sensor_read.h"                // for GPS read functions 

#include <stdio.h>
#include "ObjectDetect.h"               // Obstacle detection 


                                        /

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    TMR3_Start();                                                               //Timer used for Front Obstacle detection 
    // GPS was commented out during demo as Nav 
    // as ESP had no implemnation of GPS. 
    // I did try to correct course using course info from GPS. this failed due to slow update rate 
    // see sensor_read.c GPS function, PIC README, or report                                       
    //Init_GPS();                                                               // Configures GPS to send GPRMC sentence every second

                                                                                // Holds readings for 5 front facing sensors 
    // Structs used in front obstacle detection 
    struct Reading R1;
    struct Reading R2;
    struct Reading R3;
    struct Reading R4;
    struct Reading R5;
      
     
     // Struct for side 
     // Not implemented as all of NAV had to be rewritten 
     // and i did not have the time to implement side detection 
     // the code on the PIC does work                                     
    /*
    struct Reading Side1; 
    struct Reading Side2; 
     */
                                // this is not the pin we think it is 
                                // need to find other pin for side detect 
    
    
    GPIO_RF2_OutputEnable();    // NAV-MCU altert 
    GPIO_RF2_Clear();           // Threshold 80cm 
    

   


    
    
   
   
    while ( true )
    {   
        //BUILD AND DEPLOY IN DEBUG 
        //BUILD AND DEPLOY IN DEBUG 
        //BUILD AND DEPLOY IN DEBUG 
        
        Read_Front_Array(R1,R2,R3,R4,R5);                                       // Reads front sensors, and send obstacle distance
                                                                                // to NMCU via UART 5 
        
        //sideDetect(Side1, Side2);                                             // sideDetection not implemented during demo, see above, and summary below 
        DelayMs(100);  // 1000 -> 100                                           // Allows GPS to load, may be able to reduce
        //read_gps();                                                           // Reads and parses GPS-NEMA sentence 
        
                                      
        //BUILD AND DEPLOY IN DEBUG 
        //BUILD AND DEPLOY IN DEBUG 
        //BUILD AND DEPLOY IN DEBUG 
      
      
      
        // SUMMARY OF OPPERATION FOR PIC-32 
        //
        /* 
        General summary: 
        the Pic was originally inteneded as the sole MCU, but due to defects in the PCB, and a more dificult time 
        in deleoping code we branced to using an ESP32 in addition to the pic. 
        The Pic now performs obstalce detection and reading of GPS and transmits this to the ESP 
        
        GPS:
        configures GPS module and parses RMC sentance. can send data to ESP 32 viar UART 
        
        Obstacle Detection 
        Reads front and side array, and alerts ESP via alert pin 
        
        
        // Outputs lat, lon, speed, course to NMCU via UART5 
        //  program freezes when it looses GPS lock     // fixed 
        // OR GPS sometimes causes program freeze.      // fixed 
        // did perform stamina test with gps for ~ 90 mintes. IN DEBUG MODE  
        // cause currently unknown
        // program freeze prevented by disableing UART interrups
        // and including loop controll to prevent sensor freeze from crashing PIC 
        // 
        // OLDER ISSUES : 
        // also, loss of power to sensors causes death by exception 
        // 3/1/21 ODET notes 
        /* Obstacle detection stalls when objects are to close, especially for multiple 
         * when more than one object is ~ <20 cm from array, detect function stops outputting 
         * if it continous to output, the values are false. 
         * 
         * for multiple objects further away it detects the closer one 
         * 
         * side detect is preliminary operational, but: angle is positive for closing
         * and non closing, so need to agree on convention with JOSH
         */
        // 2/28/21 UPDATE
        /* Would crash after sending ~ lines of GPS date 
         * showing load or instruction fetch exception ( error code 4) 
         * google suggested that it's either an indexing error caused by the linker? 
         * https://www.microchip.com/forums/m1020986.aspx see here 
         * that was triggered by ISRs. 
         * since only gps would trigger this error, removed interupt form UART3 
         * (connected to GPS module), delected input capture and I2C module. 
         * Stamina test shows no crash in 45+ mintues ( and running) 
         * 
         * Program will sometimes stumble (print some word 5 times without printing data) 
         * but then recovers. 
         * 
         * REMAINING ISSUES: 
         * need to check bounds and thresholds in ODET function, 
         * need to validate ODET output. 
         * need to implement side detect 
         * need to implement alter pin to NMCU 
         * current config: 
         * UART 1 Free 
         * UART 3 Reads from GPS 
         * UART 5 output to NMCU 
         * TMR 2
         * TMR 3: 40ms Period, used for front array, max value of 1562500
         * all other PINS as GPIO 
         * 
         * P11 : RC2: S5 echo
         * P12 : RC4: S4 echo 
         * P47 : RB8: S2 echo 
         * P49 : RB10: S1 echo 
         * P110: RD2: S3 echo 
         * 
         * P59: RB12: Front Array Trigger  
         * 
         * 
         *
         * P61 : RB14 side trigger 
         * P113: RD13? side 1 echo 
         * P115: RJ1?  side 2 echo STILL NEEDS TRIGGER 
         * 
         * 
         * P38 RB7 Message alert pin 
         * 
         *  Currently Un-tasked pins: 
         * 3 pin motor connect terminal 
         * 
         * 
         * P139: RG14
         * P141: RG13
         * P143: RE3
         * formerly I2C, free, but have pullups on board
         * P95
         * P96
         */
        
        // Previous state of afaris 
        /* Obstacle detection works, outputs
         * GPS alone will output ~ 20 ish lines and then crash 
         * running both together crashes first time GPS runs 
         * suspect GPS causes memory overrun. 
         * tried to pass in buffers declared above, but this causes imediate expection 
         * 
         * NOTE obdet freezes when sensors fail, 
         * adjust bounds or include reset? 
         */
        
        
     
       
      
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

