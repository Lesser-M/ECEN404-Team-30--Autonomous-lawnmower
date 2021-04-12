# ECEN404_MCU
MCU code for senior desgin project

README OUT OF DATE 
some nav code added. will update soon 

Code for use on the PIC32MZ-2048-EFH-144 Microcontroller used for this project. 
TODO: 
get region specifiactions from josh and configure alert pin. 
Get final message format from josh ( commas, whitespaces, <CR><LF>) 
 where and if to put them. 

Includes obstacle detection for front and side, as well as GPS. 
Is to include alert pin. 

Pin will set high if an object is within a specified distance, or if the mower is closing on an object on the side 
within a specifed distance. Not yet implemented, as we are awaiting the region bounds from josh and navigation. 


Front Obstacle detection: 
Code triggers and read array of 5 sensors, aimed towards the front of the mower. 
It then determines which senser returns the shortest distance, and what side of the mower this sensor is on (left, right or center). 
It then reports out, via UART, the shortest distance to the object, as well as the side of the mower the object is on. 
All directions are referenced to the drive direction of the mower (ie the direction an observer sitting on the front of the mower would look)
All distances used herein are in cm. 

Side detection: 
works similar to front, as that it triggers all it's sensors, and then reads the distances. Uses only 2 sensors. 
One towards the front of the mower, and one towards the rear. It gets distances from both, and then calcualtes the 
minimum distance to an object on the left side of the mower, as well as if the mower is closing to that object, 
or moving away from it. 
Reports out, via UART,  minimum distance, as well as closing/not closing. 

GPS 
Configures GPS module to send RMC type nema sentence every second. 
Parses that sentence for lattitude, longitude, course and speed. 
Reports all out via UART 
