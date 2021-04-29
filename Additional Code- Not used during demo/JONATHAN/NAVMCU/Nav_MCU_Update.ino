// Include files

//

#include <stddef.h>

#include <math.h>

#include <stdlib.h>

#ifndef LAT_LON_DIST_H

#define LAT_LON_DIST_H

// -----------------------------------------> Start-New <-------------------------------------------

#include <WiFi.h>
#include <WiFiMulti.h>

//Txt files
#include <SPIFFS.h>
#include "FS.h"

WiFiMulti WiFiMulti;
String receiveData;
String idCar = "0"; // Car identification
String battery = "97"; // Percentage value of battery
String start_status; // Car status
String latitude; // Current car latitude
String longitude; // Current car longitude
String _0_latitude = "32.123456"; // Orientation coordinates
String _0_longitude = "32.123456"; // Orientation coordinates
String _1_latitude = "32.123456"; // Orientation coordinates
String _1_longitude = "32.123456"; // Orientation coordinates
String _2_latitude = "32.123456"; // Orientation coordinates
String _2_longitude = "32.123456"; // Orientation coordinates
String _3_latitude = "32.123456"; // Orientation coordinates
String _3_longitude = "32.123456"; // Orientation coordinates


SemaphoreHandle_t myMutex; // Semaphore to allow access to variable by different tasks at different times

// Method that allows to use "Serial.print ()" in different tasks
String str_global = "";
void printGlobal(String str) {
  xSemaphoreTake(myMutex, portMAX_DELAY);
  str_global = str;
  Serial.println(str_global);
  xSemaphoreGive(myMutex);
}

// ------------------------------------------- //------------------------------------------//

// Method of creating and reading files;
// Start: Create filess
void createFiles(String _file, String content) {
  //
  bool createFile = SPIFFS.exists("/" + _file + ".txt");
  if (createFile) {
    File file = SPIFFS.open("/" + _file + ".txt", "r");
    if (!file) {
      exit(0);
    }
    int s = file.size();

    String data = file.readString();

    file.close();
  } else {
    File file = SPIFFS.open("/" + _file + ".txt", "w");
    file.println(content);
    file.close();
  }
}
// Final: Create Files

// Start Methodo: Set Files of the Status and Update
// Read a String and Save into File
void setStatusFiles(String arquivo, String var) {
  bool createFile = SPIFFS.exists("/" + arquivo + ".txt");
  if (createFile) {
    File file = SPIFFS.open("/" + arquivo + ".txt", "w");
    file.println(var);
    file.close();
  }
}// Final Methodo

/*Set variables of the States and Update*/
/*Read file and return your value*/
String setStatusVar(String _file) {
  bool createFile = SPIFFS.exists("/" + _file + ".txt");
  if (createFile) {
    File file = SPIFFS.open("/" + _file + ".txt", "r");
    String var = file.readStringUntil('\n');
    //String var = file.readString();
    file.close();
    return var;
  }
}
// Final Method;

// ------------------------------------------- //------------------------------------------//


// ------------------------------------------- //------------------------------------------//
// Method to Read String containing Json
String readJson(String strJsonInput, String indexJsonInput) {
  String strJson = strJsonInput;
  String indexJson = indexJsonInput;
  String valueJson = "";
  int cont = 0;
  int cont1 = 0;
  String positionWord = "n";
  char readValue = 'n';
  while (cont < strJson.length()) {
    if (strJson.startsWith(indexJson, cont)) {
      readValue = 's';
      break;
    }
    cont++;
  }
  while (positionWord == "n") {
    if (!strJson.startsWith("\"", (cont + indexJson.length() + 3 + cont1))) {
      valueJson += strJson.charAt(cont + indexJson.length() + 3 + cont1);
      cont1++;
    } else {
      positionWord = "s";
      break;
    }
  }
  if (readValue == 's') {
    return valueJson;
  } else {
    return "false";
  }
}
// ------------------------------------------- //------------------------------------------//


// --------------------------------------> End-New <-----------------------------------------

/* Function Declarations */

extern double lat_lon_dist(double lat1, double long1, double alt1,

                           double lat2, double long2, double alt2);

#endif

#ifndef CALC_BEARING_H

#define CALC_BEARING_H

/* Function Declarations */

extern double calc_bearing( double start_lat_deg, double start_lon_deg,

                            double end_lat_deg, double end_lon_deg );

#endif

// ledPin referes to ESP32 GPIO 23

const int ledPin = 23;

// Setup

// Demonstrates use of rotary encoder for motor direction and distance

//

#define L_CHA 25

#define L_CHB 26

#define R_CHA 32

#define R_CHB 33

#define CW_LED 34

#define CCW_LED 35

volatile int L_master_count = 0; // universal count

volatile int R_master_count = 0; // universal count

volatile byte L_INTFLAG1 = 0; // interrupt status flag

volatile byte R_INTFLAG1 = 0; // interrupt status flag

// the number of the PWM pin

const uint8_t PinL  = 12;  // 12 corresponds to GPIO16 (ray changed to uint8_t on 3/6)

const uint8_t PinR  = 13;  // 13 corresponds to GPIO17 (ray changed to uint8_t on 3/6)

// setting PWM properties

const uint32_t freq = 5000;  // ray made uint32_t on 3/6

const uint8_t L_Channel = 1; // note channel 0 and 1 share frequencies

const uint8_t R_Channel = 2; // note channel 2 and 3 share frequencies

const uint8_t resolution = 8;

////////////

// SETUP  //

////////////

//

void setup() {
  // -------------------------------> Start - New <------------------------------------
  Serial.begin(115200);
  delay(10);

  // Start variable:
  String ireceiveData;
  ireceiveData = "{\"start_status\":\" \",";
  ireceiveData += "\"_0_latitude\":\" \",\"_0_longitude\":\" \",\"_1_latitude\":\" \",\"_1_longitude\":\" \",";
  ireceiveData += "\"_2_latitude\":\" \",\"_2_longitude\":\" \",\"_3_latitude\":\" \",\"_3_longitude\":\" \",";



  // Configuration of the .TXT File - Data:
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Creation of files that inform if you are sending or receiving data:
  createFiles("receiveData", ireceiveData);

  /*Initializes Variables:*/
  receiveData = setStatusVar("receiveData");
  //Test receive value
  Serial.println("receiveData :" + receiveData);

  /*Update variables reveive from EspMaster*/
  start_status =  readJson(receiveData, "start_status"); // Received from Firebase
  _0_latitude = readJson(receiveData, "_0_latitude");  // Received from Firebase
  _0_longitude = readJson(receiveData, "_0_longitude"); // Received from Firebase
  _1_latitude = readJson(receiveData, "_1_latitude");  // Received from Firebase
  _1_longitude = readJson(receiveData, "_1_longitude"); // Received from Firebase
  _2_latitude = readJson(receiveData, "_2_latitude");  // Received from Firebase
  _2_longitude = readJson(receiveData, "_2_longitude"); // Received from Firebase
  _3_latitude = readJson(receiveData, "_3_latitude");  // Received from Firebase
  _3_longitude = readJson(receiveData, "_3_longitude"); // Received from Firebase


  //Test receive value
  Serial.println("receiveData :" + receiveData);

  // We start by connecting to a WiFi network
  WiFiMulti.addAP("ESP32Server", "87654321");
  //WiFiMulti.addAP("WIFIRPJM", "761892JM");

  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);

  // Semaphore and Tasks creation
  myMutex = xSemaphoreCreateMutex();
  if (myMutex != NULL) {
    //create a task
    xTaskCreatePinnedToCore(
      TaskMain,   /* Task function. */
      "TaskMain",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      2,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(500);

    //create a task
    xTaskCreatePinnedToCore(
      TaskComm,   /* Task function. */
      "TaskComm",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      3,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(500);
  }
  // -------------------------------> Stop - New <-------------------------------------

  // LED

  pinMode( ledPin, OUTPUT);

  //Shaft encoders

  pinMode(L_CHA, INPUT);

  pinMode(L_CHB, INPUT);

  pinMode(R_CHA, INPUT);

  pinMode(R_CHB, INPUT);

  pinMode(CW_LED, OUTPUT); // LED connected to pin to ground

  pinMode(CCW_LED, OUTPUT); // LED connected to pin to ground

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(25), L_flag, RISING);

  attachInterrupt(digitalPinToInterrupt(26), L_flag, RISING);

  attachInterrupt(digitalPinToInterrupt(32), R_flag, RISING);

  attachInterrupt(digitalPinToInterrupt(33), R_flag, RISING);

  // interrupt 0 digital pin 2 positive edge trigger

  // attach the channel to the GPIO to be controlled

  ledcAttachPin(12, 1); //( PinL, L_Channel);

  ledcAttachPin(13, 2); //( PinR, R_Channel);

  // configure PWM functionalitites

  // These 2 lines were moved below ledcAttachPin by ray on 3/6

  // because this is how it was done in an online example

  ledcSetup(1, 5000, 8); //( L_Channel, freq, resolution);

  ledcSetup(2, 5000, 8); //( R_Channel, freq, resolution);




  Serial.begin(9600);



} // end void setup()


////////////////

// MAIN LOOP  //

////////////////

//

// -------------------------------> Start - New <-------------------------------------
// Task Main - "Loop"
void TaskMain( void * pvParameters ) {

  for (;;) {

    if (L_INTFLAG1) {

      delay(50);// delay in milliseconds

      L_INTFLAG1 = 0; // clear flag

    } // end if (L_INTFLAG1)



    if (R_INTFLAG1) {

      delay(50);// delay in milliseconds

      R_INTFLAG1 = 0; // clear flag

    } // end if (R_INTFLAG1)


    double  mower_width,  D0,  MAX_LEGS;

    double  r_e,  dt,  wheel_diam,  Cw;

    double  shaft_encoder_levels,  shaft_enc_ang_step;

    double  v_mow_normal_mph,  v_mow_turn_mph,  mph_to_mps;

    double  mps_to_rpms,  rpms_to_mps,  deg_to_rad, rad_to_deg;

    double  lat_start_deg, lon_start_deg,  heading_deg;

    double  L_duty, R_duty, left_delta_pos, right_delta_pos;

    double  L_rpms_desired,  R_rpms_desired, L_rpms_actual,  R_rpms_actual;

    double  v_left_actual_mps,  v_right_actual_mps;

    double  L_wheel_exact_deg,  R_wheel_exact_deg;

    double  t,  next_heading_deg, L_shaft_enc, R_shaft_enc, x, y, d;

    double  lat_deg, lon_deg, radius_turn, delta_heading_deg, dL;

    double  dx, dy, ch, sh, dxr, dyr;

    double  pi = 3.141592653589793, D_meters, wheel_radius;

    double  lat_input_deg[4], lon_input_deg[4];

    double  D_12, D_23;

    double  L_shaft_enc_prev, R_shaft_enc_prev;

    int     n, leg_number;

    int     dutyCycle;



    // CONSTANTS

    mower_width = 0.67; //meters

    D0 = 10; // Initial leg length in meters

    MAX_LEGS = 10;  // mows 10 legs for starters

    leg_number = 0; // initialization

    r_e = 6378137; // equatorial radius of earth in meters

    wheel_diam = 8 / 12 * 0.3048; // wheel diameter assumed to be 8 inches



    wheel_radius = wheel_diam / 2.0;

    Cw = pi * wheel_diam;       // wheel circumference

    shaft_encoder_levels = 500.0;

    shaft_enc_ang_step = 360.0 / shaft_encoder_levels;



    v_mow_normal_mph = 4;

    v_mow_turn_mph = 2;



    // CONVERSIONS

    mph_to_mps = 1609 / 3600; // 1609 meters in a mile and 3600 sec in an hr

    mps_to_rpms = 60 / Cw;

    rpms_to_mps = Cw / 60;

    deg_to_rad = pi / 180;

    rad_to_deg = 180 / pi;



    // Write code to accept the 4 LAT LON coreners from the app

    // here:
    xSemaphoreTake(myMutex, portMAX_DELAY);
    double testDdouble =  _0_latitude.toDouble();
    xSemaphoreGive(myMutex);

    printGlobal(String(testDdouble, 4)); // Test
    
    printGlobal(_0_latitude); // Orientation coordinates
    printGlobal(_0_longitude); // Orientation coordinates
    //printGlobal(_1_latitude.toDouble()); // Orientation coordinates
    //printGlobal(_1_longitude.toDouble()); // Orientation coordinates
    //printGlobal(_2_latitude.toDouble()); // Orientation coordinates
    //printGlobal(_2_longitude.toDouble()); // Orientation coordinates
    //printGlobal(_3_latitude.toDouble()); // Orientation coordinates
    //printGlobal(_3_longitude.toDouble()); // Orientation coordinates


    //

    //

    // Take 4 corners GPS coordinates and transform them into distances

    //



    D_12 = lat_lon_dist( lat_input_deg[0], lon_input_deg[0], 0.0,

                         lat_input_deg[1], lon_input_deg[1], 0.0);



    D_23 = lat_lon_dist( lat_input_deg[1], lon_input_deg[1], 0.0,

                         lat_input_deg[2], lon_input_deg[2], 0.0);



    L_shaft_enc_prev =  L_master_count * shaft_enc_ang_step;

    R_shaft_enc_prev =  R_master_count * shaft_enc_ang_step;



    next_heading_deg = 0; // Initialization




    ///////////////////////////////////////////////////////////////////////

    // BLINK THE LED 5 TIMES TO SHOW THAT A STRAIGHT LEG IS STARTING

    n = 0;

    while (n < 5) {

      n++;

      //digitalWrite( ledPin, HIGH);

      //delay(1000);

      //digitalWrite( ledPin, LOW);

      //delay(1000);

    }





    //////////////////////////

    // START A STRAIGHT LEG

    //

    while (leg_number < MAX_LEGS) {

      next_heading_deg += 90.0;

      //  reset distance each turn completion

      d = 0.0;

      t = millis() / 1000.0;

      leg_number++;



      //  make the leg distance smaller every so often so the the mower

      //  will go in a pattern of decreasing box sizes

      //

      if ( leg_number < 4) {

        if ((leg_number % 2) == 0) {

          D_meters = D_23;

        }

        else {

          D_meters = D_12;

        }

      }

      else {

        if ( (leg_number % 2) == 0) {

          D_meters = D_23 - mower_width * (leg_number / 2.0 - 1.0);

        }

        else {

          D_meters = D_12 - mower_width * ((leg_number - 1.0) / 2.0 - 1.0);

        }

      }



      // Pulled this out of the loop to prevent from setting the speed

      // over and over quickly (not sure if that's a problem or not)

      L_rpms_desired = v_mow_normal_mph * mph_to_mps * mps_to_rpms;

      R_rpms_desired = v_mow_normal_mph * mph_to_mps * mps_to_rpms;

      // need to send this (PWM) to the motor driver

      // Empirical formula derived from testing of mower

      //

      L_duty = (L_rpms_desired / 43.0) + 20.8; //20.0;

      R_duty = (R_rpms_desired / 43.0) + 20.8; //20.0;



      // Write out Duty cycle to interface to Motor Controller

      dutyCycle = floor(L_duty / 100.0 * 255.0 + 0.5);

      ledcWrite(1, 63); //( L_Channel, 63);  // Arduino does NOT support analogWrite()

      dutyCycle = floor(R_duty / 100.0 * 255.0 + 0.5);

      ledcSetup(2, 5000, 8); //( R_Channel, freq, resolution);

      delay(100);

      ledcWrite(2, 63); //( R_Channel, 63);  // Arduino does NOT support analogWrite()



      // Hard code D_meters for now during debugging

      D_meters = 10.0;



      //  at 90% of desired distance start turning

      while (d < 0.9 * D_meters) {

        delay(50);

        dt = millis() / 1000.0 - t;

        t = t + dt;

        //    Serial.print("hello world");

        //  Desired RPMs to set the mower at the right speed



        L_shaft_enc_prev = L_shaft_enc;

        R_shaft_enc_prev = R_shaft_enc;



        // Use the shaft encoder readings to determine how much the

        // mower has moved since the last time through the loop

        L_shaft_enc =  L_master_count * shaft_enc_ang_step;

        R_shaft_enc =  R_master_count * shaft_enc_ang_step;



        left_delta_pos  = (L_shaft_enc - L_shaft_enc_prev) * deg_to_rad *

                          wheel_radius;



        // Since the shaft encoder count is reset to zero when it

        // exceeds 500, we need to unwrap the positions by Cw

        if ( left_delta_pos < 0.0 ) {

          left_delta_pos = left_delta_pos + Cw; // wrap around

        }



        right_delta_pos = (R_shaft_enc - R_shaft_enc_prev) * deg_to_rad *

                          wheel_radius;



        if ( right_delta_pos < 0.0 ) {

          right_delta_pos = right_delta_pos + Cw; // wrap around

        }

        dL = (left_delta_pos + right_delta_pos) / 2.0;

        d = d + dL + 0.1;

        //The following two lines solve for the turn radius and the

        //rotation of the mower as a result of one wheel speeding up

        //versus the other

        radius_turn = mower_width * right_delta_pos /

                      (left_delta_pos + 0.000001 - right_delta_pos);



        delta_heading_deg = (left_delta_pos - right_delta_pos) /

                            mower_width * rad_to_deg;



        heading_deg = heading_deg + delta_heading_deg;

        if (left_delta_pos == right_delta_pos)

        {

          // Infinite turning radius means you are going straight so

          // just propagate the x and y coordinates in ground

          // coordinates by rotation into the heading direction

          x = x + dL * sin(deg_to_rad * heading_deg);

          y = y + dL * cos(deg_to_rad * heading_deg);

        }

        else

        {

          // First calculate the x and y deltas resulting from the

          // motors going different speed (and thus heading) in

          // coordinates relative to the last time interval's mower

          // position

          dx = (mower_width / 2 + radius_turn) *

               (1 - cos(deg_to_rad * delta_heading_deg));

          dy = (mower_width / 2 + radius_turn) * sin(deg_to_rad *

               delta_heading_deg);

          // Now rotate the mower coordinates into the ground

          // coordinates by rotation of heading below

          ch = cos(deg_to_rad * heading_deg);

          sh = sin(deg_to_rad * heading_deg);

          dxr =  dx * ch + dy * sh;

          dyr = -sh * dx + ch * dy;

          // Update the mower x,y location referenced to the back

          // center of the mower

          x = x + dxr;

          y = y + dyr;

        } // end else from if (radius_turn== INFINITY

      }

      /////////////////

      // MAKE A TURN //

      /////////////////



      L_rpms_desired = v_mow_normal_mph * mph_to_mps * mps_to_rpms;

      R_rpms_desired = v_mow_turn_mph * mph_to_mps * mps_to_rpms;



      L_duty = (L_rpms_desired / 43.0) + 20.8;

      R_duty = (R_rpms_desired / 43.0) + 20.8;



      dutyCycle = floor(L_duty / 100.0 * 255.0 + 0.5);

      ledcWrite(1, 63); //(L_Channel, 63);  // Arduino does NOT support analogWrite()

      //digitalWrite(13, LOW);//Remove voltage

      ledcSetup(2, 0, 8); //( R_Channel, freq, resolution);

      delay(100);

      //pinMode(13,INPUT);//Turn off right mower

      //delay(100);



      // blink LED 10 times to show that "turning off PWM on right side

      //code was surpassed

      n = 0;

      /* while(n<10){

           n++;

           digitalWrite( ledPin, HIGH);

           delay(1000);

           digitalWrite( ledPin, LOW);

           delay(1000);

        }

      */


      while (heading_deg < next_heading_deg)

      {

        delay(50);

        //NOTE: this is a clockwise turning mower so the right wheel

        //will slow down to v_mow_turn_mph below since we are turning

        // here



        L_shaft_enc_prev = L_shaft_enc;

        R_shaft_enc_prev = R_shaft_enc;



        // Use the shaft encoder readings to determine how much the

        // mower has moved since the last time through the loop

        L_shaft_enc =  L_master_count * shaft_enc_ang_step;

        R_shaft_enc = R_master_count * shaft_enc_ang_step;

        left_delta_pos  = (L_shaft_enc - L_shaft_enc_prev) * deg_to_rad *

                          wheel_radius;

        if ( left_delta_pos > 0.0001) {

          digitalWrite( ledPin, HIGH);

          delay(1000);

          digitalWrite( ledPin, LOW);

          delay(1000);

        }

        // Since the shaft encoder count is reset to zero when it

        // exceeds 500, we need to unwrap the positions by Cw

        if ( left_delta_pos < 0.0 ) {

          left_delta_pos = left_delta_pos + Cw; // wrap around

        }

        right_delta_pos = (R_shaft_enc - R_shaft_enc_prev) * deg_to_rad *

                          wheel_radius;



        if ( right_delta_pos < 0.0 ) {

          right_delta_pos = right_delta_pos + Cw; // wrap around

        }



        dL = (left_delta_pos + right_delta_pos) / 2.0;



        // The following two lines solve for the turn radius and the

        // rotation of the mower as a result of one wheel speeding up

        // versus the other

        radius_turn = mower_width * right_delta_pos /

                      (left_delta_pos + 0.000001 - right_delta_pos);



        delta_heading_deg = (left_delta_pos - right_delta_pos) /

                            mower_width * rad_to_deg;



        heading_deg = heading_deg + delta_heading_deg  + 1.0;



        if (left_delta_pos == right_delta_pos) {

          // Infinite turning radius means you are going straight so

          // just propagate the x and y coordinates in ground

          // coordinates by rotation into the heading direction

          x = x + dL * sin(deg_to_rad * heading_deg);

          y = y + dL * cos(deg_to_rad * heading_deg);

        }

        else {

          // First calculate the x and y deltas resulting from the

          // motors going different speed (and thus heading) in

          // coordinates relative to the last time interval's mower

          // position

          dx = (mower_width / 2 + radius_turn) *

               (1 - cos(deg_to_rad * delta_heading_deg));

          dy = (mower_width / 2 + radius_turn) *

               sin(deg_to_rad * delta_heading_deg);

          // Now rotate the mower coordinates into the ground

          // coordinates by rotation of heading below

          ch = cos(deg_to_rad * heading_deg);

          sh = sin(deg_to_rad * heading_deg);

          dxr =  dx * ch + dy * sh;

          dyr = -sh * dx + ch * dy;

          // Update the mower x,y location referenced to the back

          // center of the mower

          x = x + dxr;

          y = y + dyr;

        } // end else for if (radius_turn == INFININTY)

      } // end while heading_deg < next_heading_deg

      pinMode(13, OUTPUT);

    } // end while leg_number < MAX_LEGS



    vTaskDelay(110 / portTICK_PERIOD_MS);
  }
}
// -------------------------------> Stop - New <-------------------------------------

// Task Comm
void TaskComm( void * pvParameters ) {
  for (;;) {
    //    const uint16_t port = 80;
    //    const char * host = "192.168.1.1"; // ip or dns
    const uint16_t port = 80;
    const char * host = "192.168.4.1"; // ip or dns

    //Serial.print("Connecting to ");
    printGlobal("Connecting to ");
    //Serial.println(host);
    printGlobal(host);

    // Use WiFiClient class to create TCP connections
    WiFiClient client;

    if (!client.connect(host, port)) {
      //Serial.println("Connection failed.");
      printGlobal("Connection failed.");
      //Serial.println("Waiting 5 seconds before retrying...");
      printGlobal("Waiting 5 seconds before retrying...");
      delay(500);
      return;
    }

    // Sending data to the "esp32 Master"
    client.print("[{\"idCar\":\"" + idCar + "\",\"battery\":\"" + battery + "\",\"latitude\":\"" + latitude + "\",\"longitude\":\"" + longitude + "\"}]");

    int maxloops = 0;

    //wait for the server's reply to become available
    while (!client.available() && maxloops < 1000)
    {
      maxloops++;
      delay(1); //delay 1 msec
    }
    if (client.available() > 0)
    {
      //read back one line from the server
      String line = client.readStringUntil('\r');
      receiveData = line;
      setStatusFiles("receiveData", receiveData);
      //line.trim();
      //Serial.println(line);
      printGlobal(line);

      if (line.length() > 10) {
        // Guarda variáveis recebidas do "Esp32 Master"
        start_status = readJson(receiveData, "start_status"); //Test
        _0_latitude = readJson(receiveData, "_0_latitude"); //Test
        _0_longitude = readJson(receiveData, "_0_longitude"); //Test
        _1_latitude = readJson(receiveData, "_1_latitude"); //Test
        _1_longitude = readJson(receiveData, "_1_longitude"); //Test
        _2_latitude = readJson(receiveData, "_2_latitude"); //Test
        _2_longitude = readJson(receiveData, "_2_longitude"); //Test
        _3_latitude = readJson(receiveData, "_3_latitude"); //Test
        _3_longitude = readJson(receiveData, "_3_longitude"); //Test

        // Teste de variaveis
        printGlobal(start_status);//Test
        printGlobal(_0_latitude);//Test
        printGlobal(_0_longitude);//Test
        printGlobal(_1_latitude);//Test
        printGlobal(_1_longitude);//Test
        printGlobal(_2_latitude);//Test
        printGlobal(_2_longitude);//Test
        printGlobal(_3_latitude);//Test
        printGlobal(_3_longitude);//Test
      }
      line = "";
    }
    else
    {
      //Serial.println("client.available() timed out ");
      printGlobal("client.available() timed out ");
    }

    //Serial.println("Closing connection.");
    printGlobal("Closing connection.");
    client.stop();

    //Serial.println("Waiting 5 seconds before restarting...");
    printGlobal("Waiting 5 seconds before restarting...");
    delay(5000);

    vTaskDelay(110 / portTICK_PERIOD_MS);
  }
}
void loop() {

} // END OF MAIN LOOP

//

////////////////

// Functions  //

////////////////

//

// function for Encoder Channel A and Channel B updating master_count

// Hardware interrupt and Interrupt Service Request (ISR)

//

void L_flag() {

  L_INTFLAG1 = 1;

  // add 1 to count for CW

  if (digitalRead(L_CHA) && !digitalRead(L_CHB)) {

    L_master_count++ ;

    digitalWrite( ledPin, HIGH);

    delay(500);

    digitalWrite( ledPin, LOW);

    delay(500);


    if (L_master_count >= 500) {

      L_master_count = L_master_count - 500;

    }

  }

  // subtract 1 from count for CCW

  if (digitalRead(L_CHA) && digitalRead(L_CHB)) {

    L_master_count-- ;

    digitalWrite( ledPin, HIGH);

    delay(2000);

    digitalWrite( ledPin, LOW);

    delay(2000);


    if (L_master_count <= -500) {

      L_master_count = L_master_count + 500;

    }

  } // end if



} // end void L_flag()

void R_flag() {

  R_INTFLAG1 = 1;

  // add 1 to count for CW

  if (digitalRead(R_CHA) && !digitalRead(R_CHB)) {

    R_master_count++ ;

    if (R_master_count >= 500) {

      R_master_count = R_master_count - 500;

    }

  }



  // subtract 1 from count for CCW

  if (digitalRead(R_CHA) && digitalRead(R_CHB)) {

    R_master_count-- ;

    if (R_master_count <= -500) {

      R_master_count = R_master_count + 500;

    }

  } // end if

} // end void R_flag()


// Function Definitions

//

double lat_lon_dist(double lat1, double long1, double alt1, double lat2,

                    double long2, double alt2)

{

  double lat1r;

  double long1r;

  double lat2r;

  double long2r;

  double a_tmp;

  double b_a_tmp;

  double c_a_tmp;

  double a;



  /*  Equatoral Radius of the Earth WGS84 */

  /*  Flattening factor WGS84 */

  /*  LAT/LONG/ALT FOR POINT 1 */

  lat1r = lat1 * 3.1415926535897931 / 180.0;

  /*  Convert to radians */

  long1r = long1 * 3.1415926535897931 / 180.0;

  /*  Convert to radians */

  /*  LAT/LONG/ALT FOR POINT 2 */

  lat2r = lat2 * 3.1415926535897931 / 180.0;

  /*  Convert to radians */

  long2r = long2 * 3.1415926535897931 / 180.0;

  /*  Convert to radians */

  /*  POINT 1 CONVERSION TO X,Y,Z USING ELLIPSOID MODEL */

  a_tmp = sin(lat1r);

  /*  POINT 2 CONVERSION TO X,Y,Z USING ELLIPSOID MODEL */

  b_a_tmp = sin(lat2r);

  /*  DISTANCE BETWEEN TWO POINTS */

  c_a_tmp = (6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 *

                                (a_tmp * a_tmp)) +  alt1) * cos(lat1r);

  lat1r = (6.378137E+6 / sqrt(1.0 - 0.0066943799901413165 *

                              (b_a_tmp * b_a_tmp)) + alt2) * cos(lat2r);

  a = c_a_tmp * cos(long1r) - lat1r * cos(long2r);

  lat2r = c_a_tmp * sin(long1r) - lat1r * sin(long2r);

  lat1r = (6.3354393272928195E+6 / sqrt(1.0 - 0.0066943799901413165 *

                                        (a_tmp * a_tmp)) + alt1) * a_tmp - (6.3354393272928195E+6 /

                                            sqrt(1.0 - 0.0066943799901413165 * (b_a_tmp * b_a_tmp)) + alt2)

          * b_a_tmp;

  return sqrt((a * a + lat2r * lat2r) + lat1r * lat1r);

}

/* End of code generation (lat_lon_dist.c) */

// function to calculate the bearing (angle wrt north) going from one lat

// lon to another lat lon

//

double calc_bearing( double start_lat_deg, double start_lon_deg,

                     double end_lat_deg, double end_lon_deg )

{

  double pi;

  double deg_to_rad;

  double rad_to_deg;

  double start_lat_rad;

  double start_lon_rad;

  double end_lat_rad;

  double end_lon_rad;

  double dLon_rad;

  double x, y;

  double bearing_deg;

  pi = acos(-1.0);

  deg_to_rad = pi / 180.0;

  rad_to_deg = 180.0 / pi;

  start_lat_rad = start_lat_deg * deg_to_rad;

  start_lon_rad = start_lon_deg * deg_to_rad;

  end_lat_rad = end_lat_deg * deg_to_rad;

  end_lon_rad = end_lon_deg * deg_to_rad;

  dLon_rad = end_lon_rad - start_lon_rad;

  y = sin(dLon_rad) * cos(end_lat_rad);

  x = cos(start_lat_rad) * sin(end_lat_rad) -

      sin(start_lat_rad) * cos(end_lat_rad) * cos(dLon_rad);

  bearing_deg = atan2(y, x) * rad_to_deg;

  bearing_deg = bearing_deg - 360 * round(bearing_deg / 360);



  return bearing_deg;

}
