
#include <iostream> 
#include <math.h>
using namespace std; 

struct Reading { 
    float dist; 
    int senNum;
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
const float SenSep = 8.5725; // constant tracking sensor seperation, physical constant of obstacle detection array 

int main() { 

    // simulating sensor readings, to be replaced with actual readings when implemented 
    struct Reading R1; 
    R1.dist = 61.2; 
    R1.senNum = 1; 

    struct Reading R2; 
    R2.dist = 61.6; 
    R2.senNum = 2; 

    struct Reading R3; 
    R3.dist = 99.0; 
    R3.senNum = 3; 

    struct Reading R4; 
    R4.dist = 100.0; 
    R4.senNum = 4; 

    struct Reading R5; 
    R5.dist = 0.0; 
    R5.senNum = 5; 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////BEGIN DECLARATION OF VARIBLES /////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //
    ////
    ////// struct arrays holding readings at various stages of the sorting process 
    ////
    //
    struct Reading O[5] = {R1,R2,R3,R4,R5}; // sensor readings
    struct Reading O1[5] = {}; // Readings greater than D1 
    struct Reading O2[5] = {}; // Readings greater than D2 
    struct Reading O3[5] = {}; // Readings greater then D3
    struct Reading O0[5] = {}; // Readings between D1 and Do 

    struct Reading Odet[5] = {}; // Readings resolving the same obstacle 
    //
    ////
    ////// struct arrays holding readings at various stages of the sorting process 
    ////
    //
   

   //
   ////
   ////// INternal count and tracking varibles used in sorting loops and validation procedures 
   ////
   //
   
    float maxdiff = 6.0;

    int cnt = 0; // counts for sorting loops 
    int cnt2 = 0; // count for sorting loops 

    int cnt3 = 0; // loop count to move elements from O0 into detected object array 
    int cntDet = 0; // count of sensors with valid readings in the given detection range 
    //
   ////
   ////// INternal count and tracking varibles used in sorting loops and validation procedures 
   ////
   //


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
    ////// Distances defining different boundary regions, where point obstacles can be resolved with different numbers of sensors 
    ////// not including error bounds, those are added in the applicable sorting loops themselves 
    ////
    //


   //
   ////
   ////// Variables to hold results or intermediate values needed to calcualte results, to be passed to calcualtion functions 
   ////
   //
    float Dadj; // Adjacent distance, intermediate step to result 
    float Dist; // distance from mower to object, result parameter 
    float angleRef; // angle between line from refence sensor to object and front of mower, intermediate value 
    float angleNormal; // angle between front of mower and line from cneter of mower to object
    //
   ////
   ////// Variables to hold results or intermediate values needed to calcualte results, to be passed to calcualtion functions 
   ////
   //

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////END DECLARATION OF VARIBLES /////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    
    
 
    ////
    ////
    // Loops to sort readings according to boundary region they fall into 
    ////
    ////
    
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

    // Variables to track up to 5 different obstacles 
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
    // // obstacle 4 
    // float angleRef4 = 0.0; 
    // float Dist4 = 0.0; 
    // float Dadj4 = 0.0; 
    // float angleNormal4 = 0.0;
    

    ////
    // For obstacles falling into region D0 
    ////
    if (O0[0].dist != 0)
    { 
        for (cnt3 = 0; cnt3 <5; cnt3 ++)
        {
            if(O0[cnt3].dist != 0)
            {
                Odet[cnt3]= O0[cnt3]; 
                cntDet ++; 
            }
        }
        angleRef = CalcAng(Odet,cntDet); 
        Dist = CalcDist(Odet, angleRef, cntDet); 
        Dadj = CalcDadj(Odet, cntDet, angleRef); 
        angleNormal = CalcNormAngle(Dadj, Dist); 
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
                Odet[cnt3]= O1[cnt3]; 
                cntDet ++; 
            }
        }
        angleRef1 = CalcAng(Odet,cntDet); 
        Dist1 = CalcDist(Odet, angleRef1, cntDet); 
        Dadj1 = CalcDadj(Odet, cntDet, angleRef1); 
        angleNormal1 = CalcNormAngle(Dadj1, Dist1); 
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
                Odet[cnt3]= O2[cnt3]; 
                cntDet ++; 
            }
        }
        angleRef2 = CalcAng(Odet,cntDet); 
        Dist2 = CalcDist(Odet, angleRef2, cntDet); 
        Dadj2 = CalcDadj(Odet, cntDet, angleRef2); 
        angleNormal2 = CalcNormAngle(Dadj2, Dist2); 
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
                Odet[cnt3]= O3[cnt3];
                cntDet ++; 
            }
        }
        angleRef3 = CalcAng(Odet,cntDet); 
        Dist3 = CalcDist(Odet, angleRef3, cntDet); 
        Dadj3 = CalcDadj(Odet, cntDet, angleRef3); 
        angleNormal3 = CalcNormAngle(Dadj3, Dist3); 
    } 

    // ////
    // // For obstacles falling into region D4
    // ////
    // cnt3 =0 ; 
    // if (O4[0].dist != 0)
    // { 
    //     for (cnt3 = 0; cnt3 <5; cnt3 ++)
    //     {
    //         if(O4[cnt3].dist != 0)
    //         {
    //             Odet[cnt3]= O0[cnt3]; 
    //             cntDet ++; 
    //         }
    //     }
    //     angleRef4 = CalcAng(Odet,cntDet); 
    //     Dist4 = CalcDist(Odet, angleRef4, cntDet); 
    //     Dadj4 = CalcDadj(Odet, cntDet, angleRef4); 
    //     angleNormal4 = CalcNormAngle(Dadj4, Dist4); 
    // } 


    cout <<endl << endl <<  "Object in regio Do: " << endl; 
    cout << "distance: " << Dist << endl << "angle: " << angleNormal << endl; 
    cout << "Dadj: " << Dadj << endl <<  "refAngle: " << angleRef << endl; 

    cout << endl << endl <<  "Object in regio D1: " << endl; 
    cout << "distance: " << Dist1 << endl << "angle: " << angleNormal1 << endl; 
    cout << "Dadj: " << Dadj1 << endl <<  "refAngle: " << angleRef1 << endl; 


    cout << endl << endl << "Object in regio D2: " << endl; 
    cout << "distance: " << Dist2 << endl << "angle: " << angleNormal2 << endl; 
    cout << "Dadj: " << Dadj2 << endl <<  "refAngle: " << angleRef2 << endl; 


    cout << endl << endl << "Object in regio D3: " << endl; 
    cout << "distance: " << Dist3 << endl << "angle: " << angleNormal3 << endl; 
    cout << "Dadj: " << Dadj3 << endl <<  "refAngle: " << angleRef3 << endl; 


    // cout << endl << endl << "Object in regio D4: " << endl; 
    // cout << "distance: " << Dist4 << endl << "angle: " << angleNormal4 << endl; 
    // cout << "Dadj: " << Dadj4 << endl <<  "refAngle: " << angleRef4 << endl; 


  
    return 0; 
};

/////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS 
/////////////////////////////////////////////////////////////////////////////////





float CalcAng (struct Reading OValid[5], int cntDet )
{
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
    float Nseps = 0.0; // NUmber of integer multiple sensor seperations 
    float Fseps = 0.0; // Fraction of full sensor seperation for Dadj
    // cross center calculation, need to adjust Dadj and account for angle referance

    /* for cases where outer most resolving sensors sit on opposite sides of the center line 
    we need to take the distance from one sensor to the center, and then the distance 
    from the normal line to the sensor providing us the angle referance 
    and subtract the 2, giving us the distance from the normal line intersect to the center of the mower
    these are the 1st 2 cases. 

    When both sensors are on the same side of the mower, up to and including the center sensor 
    we need to add the 2 numbers instead to get the distance from normal line to center line
    (3rd case)
    */ 
    if ( (OValid[cntDet-1].senNum > 3) & (OValid[0].senNum < 3) )
    {
        // Full sensor seperations between center off array 
        // and point where normal line from obstacle lands 

        Nseps = (OValid[cntDet-1].senNum - 3) * SenSep; // distance from far sensor to center line 
        Fseps = cos((3.141/180)*angle)*OValid[cntDet-1].dist; // distance from normal line to reading sensor 
        Dadj = Fseps - Nseps; // distance from normal line to center 
        

    }
    // cross center calculation also, but the other way 
    else if ( (OValid[cntDet-1].senNum < 3) & (OValid[0].senNum > 3))
    {
        Nseps = (OValid[0].senNum - 3) * SenSep; // distance from far sensor to center 
        Fseps = cos((3.141/180)*angle)*OValid[cntDet-1].dist; // horizontal distance from normal line to sensor 
        Dadj = Fseps - Nseps; // distance from normal line to center 


    }
    else if ( ( (OValid[cntDet-1].senNum>=3) & (OValid[0].senNum >= 3) ) | ( (OValid[cntDet-1].senNum <=3) & (OValid[0].senNum<=3) ))
    {   // integer num of sensor seperations if both sensors are on the same side of the center, 
        // or include center sensors 
        Nseps = (fabs(OValid[cntDet-1].senNum - 3)) * SenSep;
        Fseps = cos((3.141/180)*angle) * OValid[cntDet-1].dist;   
        Dadj = Fseps + Nseps; 
    }

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
