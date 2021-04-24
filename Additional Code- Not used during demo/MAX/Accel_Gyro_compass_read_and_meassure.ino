/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low magnetometer data registers.
They can be converted to units of gauss using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).
Example: An LIS3MDL gives a magnetometer X axis reading of 1292 with its
default full scale setting of +/- 4 gauss. The GN specification
in the LIS3MDL datasheet (page 8) states a conversion factor of 6842
LSB/gauss (where LSB means least significant bit) at this FS setting, so the raw
reading of 1292 corresponds to 1292 / 6842 = 0.1888 gauss.
*/

#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

LSM6 imu;

char report[80];

LIS3MDL mag;

//char report[80];
float Ang_y_i = 0.0; 
float Ang_x_i = 0.0;  

float Ang_y = 0.0; 
float Ang_x = 0.0; 
void setup()
{
  Serial.begin(115200);
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();
  
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  Serial.println(" Lets GOOOOOOOOOOOOOOOOOOOOOOOOO" ); 
}

   //Heading = 0.0; 


void loop()
{
  
  //mag.read();

  //snprintf(report, sizeof(report), "M: %6d %6d %6d",
  //  mag.m.x, mag.m.y, mag.m.z);
  //Serial.println(report);

  //delay(2000); 
  // mag readings in mili Gauss 
  //float My = mag.m.y/6.842; 
  //float Mx = mag.m.x/6.842;
  //float Mz = mag.m.z/6.842; 
  //float Heading = 180*atan2(mag.m.y, mag.m.x)/3.141; 
  //Serial.printf("%f ", Heading);
  //Serial.println(" "); 
  //delay(1500);
  /*
  float Ang_y = 0.0; 
  float Ang_x = 0.0;
  float Ang_z = 0.0;
  bool turn = true; 
  */ 

  /*
  while (turn) 
  {
    //float Ang_y_i = Ang_y; // current angle is previous angle + angle travled over last itteration 
    float Ang_x_i = Ang_x; 
    //float Ang_z_i = Ang_z; 
    float CyC_i = ESP.getCycleCount(); //esp_timer_get_time(); // ESP.getCycleCount();  // sys clock at begin 
    imu.read();
    float CyC_F = ESP.getCycleCount(); //esp_timer_get_time(); // ESP.getCycleCount();  // sys clock after imu reading 
    //if (CyC_F > CyC_i)
    //{
    float dt = (CyC_F - CyC_i) / (80000000); // time elapsed (1000000); //  / 
    //float Ang_y = Ang_y_i + imu.g.y*dt; // gyro output * time = angle 
    float Ang_x = Ang_x_i + imu.g.x*dt;
    //float Ang_z = Ang_z_i + imu.g.z * dt; 
    //}
    */
    /*
    snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
      imu.a.x, imu.a.y, imu.a.z,
      imu.g.x, imu.g.y, imu.g.z);
    */
    /*
    //Serial.printf("x ang = %f", Ang_x);
    //Serial.println(" ");
    //Serial.println(report);
    //Serial.printf("X ang = %f, Y ang = %f Z ang = %f", Ang_y, Ang_x, Ang_z);
    //Serial.printf("X %d, Y %d ,Z %d", imu.g.x, imu.g.y, imu.g.z); 
    
    //Serial.println(" "); 
    //delay(500); 
    if ((Ang_x >= 90.0) | ( Ang_x <= -90.0)) // 90 deg angle compleated 
    { 
      turn = false;
      Serial.printf("x ang = %f", Ang_x); 
      //Serial.printf("X ang = %f, Y ang = %f Z ang = %f", Ang_y, Ang_x, Ang_z); 
    }
    
    
  }
  */
  
  //Serial.printf("X ang = %f, Y ang = %f", Ang_y, Ang_x); 
  //Serial.println(" ");
  //Serial.println(" Compleated 90 deg"); 
  //Serial.println(" ");
  //delay(2000); 
  /*

  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
    imu.a.x, imu.a.y, imu.a.z,
    imu.g.x, imu.g.y, imu.g.z);
    Serial.println(report);
    */
    // acclerations in mili g 
    mag.read();
    float My = mag.m.y/6.842; 
    float Mx = mag.m.x/6.842;
    float Mz = mag.m.z/6.842;
    imu.read();
    
    float A_X = imu.a.x *0.061;     
    float A_Y = imu.a.y * 0.061; 
    float A_Z = imu.a.z * 0.061; 
    float Pitch = 180*atan(A_X/(sqrt((A_Y * A_Y) + (A_Z*A_Z))))/3.141 ; 
    float Roll = 180*atan(A_Y/(sqrt((A_X * A_X) + (A_Z * A_Z))))/ 3.141; 
    float Yaw = 180*atan(A_Z/(sqrt((A_X*A_X) + (A_Y*A_Y))))/3.141; 
    Serial.printf(" Pitch: %f, Roll : %f, Yaw : %f", Pitch, Roll, Yaw); 
    Serial.println(" "); 

    float cos_roll = cos(Roll); 
    float sin_roll = sin(Roll); 
    float cos_pitch = cos(Pitch); 
    float sin_pitch = sin(Pitch); 

    float MAG_X = Mx*cos_pitch + My*sin_roll*sin_pitch +Mz*cos_roll*sin_pitch; 

    float MAG_Y = My*cos_roll - Mz*sin_roll; 

    float Heading = (180/3.141)*atan2(-MAG_Y, MAG_X); 

    Serial.printf(" Heading : %f", Heading); 
    Serial.println(" "); 
    

    

    
    
    
  //Serial.println(report);

  delay(1000);
  
  

  //delay(2500);
  
}
