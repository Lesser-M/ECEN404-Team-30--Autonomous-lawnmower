/* ************************************************************************** */
/** Sensor read function header 

 Max Lesser for ECEN capstone  

  @File Name
    sensor_read.h
    holds dependencies for sensor_read.c GPS read function 

  @Summary
 Declaring and defining functions to read sensor input for ECEN403 
 * MCU project 
 * 
 * 1.) Read Ultrasonic sensor1 : Read_US_1

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "definitions.h"    // device pins 
 
#include "device.h" //#include <cp0defs.h> includes CP0 definitions, for reading core timer 
/* TODO:  Include other files here if needed. */
#include <string.h>     // string library, needed for NEMA parsing 

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


#define GetSystemClock() (SYS_CLOCK) // get system clock function 

   


    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

 


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
 
    
    int DelayMs(unsigned long int msDelay);      // Custom mili second delay function 
    int DelayUs(unsigned long int usDelay);      // Custom micro second delay function 
    void Init_GPS(void);                         // Configures GPS ( Parallax 28203) 
    void read_gps(void);                         // Reads and parses GPS, can output via UART 
   
    
    
//#define RMC_sen "$GPRMC" defined in .c file 
//#define GGA_sen "$GPGGA" removed GGA capability 
    
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
