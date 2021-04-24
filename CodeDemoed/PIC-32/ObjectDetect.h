
/** Descriptive File Name

  @Company
    Written for ECEN Capstone F2020-S2021 By Max Lesser  

  @File Name
    ObjectDetect.h
    holds dependencies for ObjectDetect.c

  @Summary
    structs and dependecies for obstacle detection source code 

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */
#include <math.h>           // for trig functions 

#include "definitions.h"    // pin defintions 
 
#include "device.h"

#include "sensor_read.h"    // for delay functions 

#include <stdio.h> // converts float to char 


// Struct to hold readings reported by sensor
// Value caltulated by sensor read function 
struct Reading { 
    float dist; // distance, in cm, reported by sensor 
    int senNum; // sensor referance number, starting at 1 for left most and ending at 5 for right most, when viewed in drive direction 
}; 

// struct to hold discoverd obstacles 

struct Obstacle {

    float distance; // distance to obstacle, perpendicular to axel of mower
    float angle; // angle a line from front center of mower drawn to the obstacle makes with a line normal to the front axis of the mower 

};
float CalcAng (struct Reading O[5], int cntDet ); 
// angle calcuating function, reveiving array of up to 5 grouped, valid sensor readings 
// and an integer giving the number of valid readings within the array 

float CalcDist(struct Reading OValid[5], float angle, int cntDet);
// given the intermediate angle calcuated above and sensor number and reading 
// used for those calcuations this function finds the distance from the front of
// the mower to the object 

float CalcDadj(struct Reading OValid[0], int cntDet, float angle); 
// Calcuates adjecent side for right triangle trig purposes 
// see function definition for detailed explanation 

float CalcNormAngle(float Dadj, float dist);
// calcuates angle between line to object from center of mower 
// and front of mower


int detect(struct Reading F1, struct Reading F2, struct Reading F3, struct Reading F4, struct Reading F5); 
// performs actual obstacle detection, giving distance and angle, using above functions 

int sideDetect(struct Reading S1, struct Reading S2); 
// side facing array, reads 2 sensors on side, finds distance to side object
// and if we're closing with it or not 

int Read_Front_Array(struct Reading R1, struct Reading R2, struct Reading R3, struct Reading R4, struct Reading R5); // front facing US array read function 
// reads 5 sensors in front facing array 
//////
////////// CONSTANTS ////////////////////////////////////////////////////////////////////////////////////////////////// MAY NEED TO CHANGE THIS FOR PIC IMPLEMENTATION 
//////
#define SenSep 8.5725 // constant tracking sensor seperation, physical constant of obstacle detection array 
#define maxdiff 6.0 // maximum difference between distances reported by sensors resolving the same (point) object
// used in defining region bounds, DOUBLE CHECK FOR LEEWAY w. AREA OBJECTS  







