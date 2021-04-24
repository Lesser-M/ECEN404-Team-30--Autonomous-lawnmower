/* ************************************************************************** */
/** sensor read functions 
 * 
 * contains: 
 *  read GPS 
 * init gps 
 
 * 
 * and delay functions 
 * 
 depends on 
 sensor_read.h 

  @Company
 ECEN403-904 
 * Max Lesser 

  @Summary
    Files to read and parse sensors other than obstacle detection involved sensors 

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
#include "sensor_read.h"      // holds all dependencies  





int DelayMs(unsigned long int msDelay)
    {
        // simple mili-second delay function 
        // using system clock as reference 
        // referance links 
        // https://www.microchip.com/forums/m753509.aspx
        // https://www.microchip.com/forums/m425906.aspx
        
        // discription 
        // divide sys clock by 2 for num ticks per second
        // and divide by 1000 for num ticks per ms 
        // multiply by ms delay needed 
        // CLK Freq must be 200MHz or adjust eqn 
        unsigned long int wait_time;
        unsigned long int start; 
        unsigned long int stop; 
        // _CP0_GET_COUNT() read CP0 count register, included in device.h
        wait_time= (100000)*msDelay; 
        start = _CP0_GET_COUNT(); 
        stop = start+wait_time; 
        while ( _CP0_GET_COUNT() < stop)
        {
            
        }
        return 0; 
    };
    
    
    int DelayUs(unsigned long int usDelay)
    {
        // simple microsecond delay function 
        // using system clock as reference 
        // referance links 
        // https://www.microchip.com/forums/m753509.aspx
        // https://www.microchip.com/forums/m425906.aspx
        
        // discription 
        // for sys clk = 200MHz, -> 100k cycles per ms 
        // divide sys clock by 2 for num ticks per second
        // and divide by 10^6 for num ticks per us 
        // multiply by ms delay needed 
        
        unsigned long int wait_time = 0;
        unsigned long int start = 0; 
        unsigned long int stop = 0; 
        
        wait_time= (100/1)*usDelay; 
        start = _CP0_GET_COUNT(); 
        stop = start + wait_time; 
        
        while ( _CP0_GET_COUNT() <stop)
        {
            
        }
        return 0; 
    };
    
    

    
    void Init_GPS(void) 
    { 
        /* Function to send setup command to GPS module 
         * to configure for output of $GPRMC sentence every 1 second
         * 
         * 
         * we write the config string to uart, and wait for the configuration 
         * to take effect 
         * // once every second 
         * $PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n
         * // once every 3
         * $PMTK314,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*13\r\n
         * once every 5 seconds 
         * $PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*31\r\n
         */
     
        UART3_Write("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n", 53); 
        DelayMs(1000); // waiting for config to take hold 
          
     };
    
    void read_gps(void)
    {
        
        /* This function reads data from the GPS module on UART3, parses the 
         * received data, and sends it out via UART5
         * NOTE: 
         * GGA capacity removed to simplify, look in 2/28/21 backup for GGA 
         * parser function if need be, also 403 backups include full RMC and GGA parser 
         * It is setup to parse RMC and GGA sentences, but should only 
         * ever need to parse RMC sentences, as we configure the GPS module to 
         * only send those above. 
         * 
         * It receives data via UART 3, reads it into a buffer
         * loops through the buffer until it finds a NEMA ID 
         * it then compares the NEMA id to those we want (RMC & GGA) 
         * if it finds one of those it moves on to the parsing function 
         * where we loop through the buffer and read data fields based on 
         * the comma deliminators used in NEMA0183 format. 
         * If we do not find a sentence we want, we continue looping through 
         * the received buffer until we do, or it is empty. 
         * we print the date we found to UART5 as 
         * 
         * lat , lon , speed , course /n/r
         * 
         * dd.dddddd , ddd.dddddd , v.vv , ccc.cc
         * NEED TO FORMAT FOR DECIMAL 
         */
        ///////////////////////////////////////////
        // reading GPS
        char Read_buffer[256];                   // Read Uart into this for processing 
     
        UART3_Read(Read_buffer, 255); 
        DelayMs(500); 
        //////////////////////////////////////////
        // internal vars and counts 
        
                                                // These are the NEMA id's we are looking for 
                                                // we will compare the ID we receive with these 
                                                // and then parse based on the sentence we received 
        const char RMC_sen[7] = "$GNRMC";  
        // DO NOT RESET OR CHANGE, KEEPS TRACK OF FULL BUFFER 
        int cnt1 = 0;                           // count to loop through received buffer 
                                                // NEMA type count, used to fill type buffer ( below)
        int cnt = 0; 
        
                                                // We read the ID of the received sentence into this buffer 
                                                // to compare with the IDs we are looking for 
        char NEMA_Type[7] = "NEMA01"; 
        
                                                // data field into which to parse latitude info
                                                // lat_cnt is used to loop through this nema field 
                                                // could replace lat_cnt and those below with one varible 
                                                // so long as we rest, but if it ain't broke, don't fix it
        char lat[10]; 
        memset(lat, ' ', 10);                   // set latitude to spaces, to prevent from sending trash if we don't fill it  
        int lat_cnt = 0; 
        
                                                // data field into which to parse longitude info
                                                // lon_cnt is used to loop through this nema field 
                                                // see lat section for more detail 
        char lon[11];  
        memset(lon, ' ', 11); 
        int lon_cnt = 0; 
        
                                                // data field into which to parse speed info
                                                // speed_cnt is used to loop through this nema field 
        char speed[5];                          // speed field ( speed in knots), see lat for more info 
        memset(speed, ' ', 5); 
        int speed_cnt = 0;
        
                                                // data field into which to parse course info
                                                // crs_cnt is used to loop through this nema field 
        char course[7];                         // course field, and, you guessed it, see lat for more info 
        memset(course, ' ', 7); 
        int crs_cnt; 
        
                                                // comma_cnt counts to commas in the recieved sentence
                                                // the commas act as deliminators between the fields
                                                // using this variable we can track what data field of the received 
                                                // sentence we are in 
        // TRACKS WHAT DATA FIELD WE ARE IN, DO NOT CHANGE OR RESET  
        int comma_cnt = 0; 
        
                                                // bools to tell us if we are still reading the received buffer
                                                // and if we still need its NEMA type
                                                // controls what loop we fall into, and when we break from it 
        bool Reading = true;                    // are we still reading? 
        bool need_type = true;                  // do we still need the NEMA type? 
        
      
        
        // loop to parse  identifier 
        // Loops until the beginning of ID and parses ID 
        // Fall into Loop only if we need the identifier, 
        // And have enough left in the buffer to find the data we need 
        
        
       //*****************************************************************************************************************************//
        while( (need_type) && (cnt1<=190) )                 // loop looking for NEMA identifier 
        {
            if( Read_buffer[cnt1] == '$')                   // marks beginning of NEMA sentence
            {
                while ( Read_buffer[cnt1] != ',')           // looping through id field 
                {
                    NEMA_Type[cnt] = Read_buffer[cnt1]; 
                    cnt1++;                                 // need to incriment seperate counts, one for whole received buffer  
                    cnt++;                                  // and one for the NEMA type varible 
                }
                if ( (strcmp(NEMA_Type, RMC_sen) == 0))     // removed as we stoped using GGA | (strcmp(NEMA_Type, GGA_sen) == 0 ))
                {
                                                            // If the received ID is one we can use, move to parser 
                    need_type = false; 
                }  
                else
                {                                           // else reset NEMA_type Count and keep looking
                    cnt = 0;    
                }
                
            }
            else                                            // If we don't have ID marker, keep looping 
            {
                cnt1++; 
            }
        } 
      //*****************************************************************************************************************************//
        
        /*
         * 
         * RMC PARSER
         * WARNING LOOKS FOR GNRMC, MAY RECEIVE GPRMC OR SIMILAR 
         * need to adjust ID loop if this becomes an issue 
         * 
         * Fall into this block if the recieved id is for a RMC sentence 
         * and we have not yet parsed a full sentence 
         * parse NEMA sentences, looking for comma deliminators 
         */
        
        if (( strcmp( NEMA_Type, RMC_sen ) == 0 ) && ( Reading)) 
        {
            while((Reading) && ( cnt1<=255))                // loop while we are reading data from sentence, and there is sentence left  
            {
                if( Read_buffer[cnt1] == ',' )              // find comma deliminated fields 
                {
                                                            // if we find a comma we have found a new data field 
                                                            // in that case we move on to check if it is a field we need data from 
                                                            // if yes we read that data, otherwise we loop till the next comma 
                    comma_cnt++; 
                    cnt1++; 
                }
                    // after the 3rd comma, Latitude field, check lat_cnt 
                    // to make sure we don't fall into this twice 
                    if ( ( comma_cnt == 3 ) && (lat_cnt == 0))  // latitude field
                    {
                        lat_cnt = 0;                        // make sure lat field is filled from beginning 
                        while( (Read_buffer[cnt1] != ',') && (lat_cnt<=11) ) // loop until end of field 
                        {
                            lat[lat_cnt] = Read_buffer[cnt1];   // copy latitude to lat variable, as string 
                            cnt1++;                             // advance sentence count and Latitude field count 
                            lat_cnt++;
                        }   
                        comma_cnt++; // INCREASE COMMA COUNT!  above loop only breaks cause we found one  
                    }
                    // 4th filed is direction (N/S) skipping that to keep things simple 
                    // after the 5th comma we start the longitude field 
                    // checking lon_cnt to only do this once 
                    else if (( comma_cnt == 5 ) && (lon_cnt == 0 )) // longitude field 
                    {
                        lon_cnt = 0; 
                        while ( (Read_buffer[cnt1] != ',' ) && (lon_cnt<=10)) // looping through longitude field 
                        {
                            lon[lon_cnt] = Read_buffer[cnt1];           // copying to lon variable  
                            cnt1++; 
                            lon_cnt++;
                        }
                        comma_cnt++;            // again advancing comma cnt, cause we find one above
                    }
                    else if (comma_cnt == 7)    // getting speed in kts
                    {
                        speed_cnt = 0; 
                        while ((Read_buffer[cnt1] != ',') && (speed_cnt <=4))   // looping through speed field 
                        {
                            speed[speed_cnt] = Read_buffer[cnt1];               // copying speed to our speed variable 
                            cnt1++; 
                            speed_cnt++; 
                        }
                        comma_cnt++;                                            // We terminate on comma, so we have to count it here
                    }
                    else if (comma_cnt == 8) // getting course, ddd.dd format 
                    {
                        crs_cnt = 0; 
                        while ((Read_buffer[cnt1] != ',') && (crs_cnt <=6)) // looping through course field 
                        {
                            course[crs_cnt] = Read_buffer[cnt1]; 
                            cnt1++; 
                            crs_cnt++;     
                        }
                        comma_cnt++; // guess what? we terminate on comma 
                        Reading = false; // setting reading to false. we have the data we need 
                    }      
                cnt1++; // If we are not in a field we need, we keep looping 
            }   
            
        }
        // END OF RMC PARSER 
        // we have now extracted all the data we need from an RMC sentence 
      //*****************************************************************************************************************************//
      //*****************************************************************************************************************************//
      /*NOTE: 
      // We intended to send all or part of this data to the Navigation MCU, to use in Nav
      // I never received any specifics on how to format, and at the end i had to 
      // compleatly rewrite NAV code and rebuild Motor subsystem from scratch 
      // As well as the vast majority of hardware integration (Vincent helped me on mutliple occasions, 
      // and Josh helped for an afternoon) 
      // I did not have time to incorporate this into my frantic rewrite of NAV code. 
      // this code on the Pic thouhg works, and I did use it to read cource from the GPS onto the ESP 
      // in an attempts to correct NAvigation errors. The slow update rate of the GPS, and the fact that no time
      // was remaining in the semester at that point prevented me from successfully using GPS info for course correction 
      */ 
      //*****************************************************************************************************************************//
      //*****************************************************************************************************************************//
        
       // Latitude filed 
//        UART5_Write(lat,9); 
//        DelayMs(100);           // THIS NEEDS TO BE LONG!!! SHORT ONES CAUSE RANDOM SYMBOLS   
//        UART5_Write(" , ",3);
//        DelayMs(50);            // NEED DELAYS TO ALLOW UART TO SEND 
//        // Longitude field 
//        UART5_Write(lon,10);      
//        DelayMs(100);           // IT WILL MESS UP WITHOUT THESE 
//        UART5_Write(" , ",3);
//        DelayMs(50);            // YOU CAN TRY TO MAKE IT SHORTER IF YOU MUST
//        // speed field 
//        UART5_Write(speed,4);
//        DelayMs(100);           // BUT BE PREPARED FOR THAT TO BREAK THINGS 
//        UART5_Write(" , ",3);
//        DelayMs(50);            // FOR REAL DUDE, DON'T DO IT 
        // course field 
        DelayMs(100);
        UART5_Write(course,6); 
        DelayMs(100);           // I'VE WARNED YOU 
        UART5_Write("\n\r",2); 
        DelayMs(50);            // CAN'T SAY YOU AIN'T BEEN WARNED 
        
        
    
        
        

      //*****************************************************************************************************************************//
      
    /* UPDATE: final state: this code works. below left as resource 
     * 
     *
     * from 403, issues fixed, left here incase issues return 
     * GPS read function works 
     * We are having issues with the GPS module not sending data, 
     * // not getting a position Fix 
     * 
     * to code above will parse a test sentence, and a received buffer if 
     * the buffer is not empty
     * 
     * suspect GPS issues are due to us reading it inside, and should improve 
     * when we move outside. If the issue presists after moving outside 
     * suggest using extra GPIO port and GPS enable function to power cycle 
     * the module when the read buffer is empty.  
     * 
     * 
     * FOR REFERANCE PURPOSES 
     *     NEMA DATA FIELDS FOR GPRMC FORMAT
         * field[0] = UTC time 
         * field[1] = Status (A= valid, V= invalid) 
         * field[2] = Latitude in dd.mmm.mmmm format
         * field[3] = N or S indicator 
         * field[4] = Longitude in dddmm.mmmm format
         * field[5] = E or W indicator 
         * field[6] = speed over ground in knots
         * field[7] = course over ground in degrees
         * field[8] = date (ddmmyy) 
         * field[9] = magnetic vairation E or W
         * field[10] = East/West indicator 
         * field[11] = Mode (Autonomus or DGPS)
         * field[12] = checksum 
         * field[13] = message termination (<CR><LF>) 
     * 
     * REFERANCES 
     * 
     * NEMA format explantanion and GPS module interface sentences
     * https://www.parallax.com/sites/default/files/downloads/28504-MT3337-Platform-NMEA-Message-Spec-GPS-GLONASS_V1.00.pdf
         
     */    
    };
    
    // NOTES FORM REMOVED FUNCTIONS ARE LEFT HERE, INCASE WE WANT/ NEED TO RE-IMPLEMENT THEM 

//         A&G // Find protocol and address details for Gyroscope here 
//        //https://www.pololu.com/file/0J1087/LSM6DS33.pdf
//        //
//        // LSM6DS33 (Gyroscope) address is 110101xb
//        // X-pitch output register ADR = 0x22 



//        ISR? //https://www.microchip.com/forums/m858144.aspx
//        /*Read last A & B and compare to current A & B
//         * if A leads B one direction 
//         * if B leads A other dire 
//         * how often to sample A&B???
//         * get reference value from PWM input? 
//         * How to read I? 
//         * COMPARE BOTH SE DATA IN DIFFERENT FUNCTION??? 
//         * Return current RPM (averaged over what? 
//         * Return current direction as flag 
//         * Channel A leads B for CW rotation, viewed from top of sensor 
//         */
//   
///* *****************************************************************************
// End of File
// */
