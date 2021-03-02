/*******************************************************************************
  Main Source File
 * so meine freunde
  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
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
#include "sensor_read.h"

#include <stdio.h>
#include "ObjectDetect.h"


                                        // only for code test purposes 

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
    Init_GPS();                                                                 // Configures GPS to send GPRMC sentence every second

                                                                                // Holds readings for 5 front facing sensors 
    struct Reading R1;
    struct Reading R2;
    struct Reading R3;
    struct Reading R4;
    struct Reading R5;
    struct Reading Side1; 
    struct Reading Side2; 

   


    
    
   
   
    while ( true )
    {   
        
        
        Read_Front_Array(R1,R2,R3,R4,R5);                                       // Reads front sensors, and send obstacle distance
                                                                                // and angle to NMCU via UART 5 
        
        sideDetect(Side1, Side2); 
        DelayMs(1000);                                                          // Allows GPS to load, may be able to reduce 
        read_gps();                                                             // Reads and parses GPS-NEMA sentence 
                                                                                // Outputs lat, lon, speed, course to NMCU via UART5 
        //  program freezes when it looses GPS lock 
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

