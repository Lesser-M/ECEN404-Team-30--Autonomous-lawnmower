# ECEN404_MCU
Project wide Readme
**************************************************************************************************************
***Project info:***
-Project Title: Autonomous Lawnmower 

          -Team Number: 30 
          -Team Members: Max Lesser, Vincent McMasters, , Jonatahan Poulose, Joshua Samaniego
          -Sponsor: Stavros Kalafatis 

          -403 TA: Pranav Duphila 
          -403 Professor: S. Kalafatis 
          -404 TA: Ahmad Darwash 
          -404 Professor: J. Lusher

Problem Statement from Sponsor: 
" You will start with a lawn mower shell, add motors to propel the wheels, microcontroller to control everything, 
comms to a wifi network where area to be covered and route will be entered, and a power mechanism (docking station or other)”

**************************************************************************************************************
***GITHUB structure*** 
This Repository contains all of our final reports, code and any other items we are submitting in relation to the final report. 
The repo contains the following folders and sub folders, with purpose as listed below


-CodeDemoed: Contains the code running on the mower at demo time 

          -/APP-Code: Contains the code for the android app connected to the server; APK file, Java Files, and XML Files 
          
          -/ESP-32: contains code running on ESP32 at demo time, nameley code for Navigation, WiFi connectivity to get coordinates and start_status, coordinates to distances                       calculation, and sends battery info to server
          
          -/PIC-32: Contains code running on the PIC-32 during demo time, Nameley code for obstacle detection 
          
          -/Server: contains any code running on the server needed to connect the ESP32 to the User interface 
         
         
-Additiona Code- Not used during Demo: contains code written for this project that was not implemnted at demo time; previous versions

          -/JONATHAN: code written by Jonathan that was not used during demo time. This includes folders with code that connects both ESPs together using WiFi as well as                         receiving/sending coordinates, start_status, and battery information to/from each other. 
          
          -/JOSH: code written by Josh that was not used during demo time 
          
          -/MAX: code written by max that was not used during demo time 
           
           
-Reports-404: contains all written Reports for this semester 
 
 
-Videos: contains demo video, validation videos, and other videos recorded that is relevant to the report

-PicturesandScreenshots: contains validation pictures & screenshots and other images recorded that is relevant to the report

 
 
each of these directories has a README file expalining it further, along with the purpose of any child directories not mentioned here
The README shall clearly state who is responsible for the code. Detailed explanations are reserved for the report in the Reports-404 section 

        

**************************************************************************************************************
***Project Summary***
We used an old lawnmower shell, stripped it of all its parts, added Motors, powersupply, wiring, Microcontroller and sensors to it to achive our mission. 

The code presented here is for 2MCUs, the PIC-32 used for GPS and obstacle detection, as well as the ESP32 used for user interface and Navigation.
Additionally presented is Code for the app, and other interface structure, along with code developed for this project but not used during demo. 

As the main project report is presented elsewhere, we will not go into detail here, but instead refer the reader to the READMEs in the appropriate sections, and the Report 








