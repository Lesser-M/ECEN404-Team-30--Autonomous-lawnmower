//#ifdef ESP32
#include <WiFi.h>
#include <HTTPClient.h>
#include <FirebaseESP32.h>
//#include <FirebaseJson.h>

//Txt files
#include <SPIFFS.h>
#include "FS.h"

#include <Arduino.h>

// Set web server port number to 80 - SoftAP
WiFiServer server(80);

// ESP32 wifi network connection data
#define ssidEspServer "ESP32Server"
#define PassEspServer "87654321"

// Connection data to the user's local wifi network
#define ssidWiFiLocalSet "Poulose"
#define passwordSenhaWiFiLocal "2147707702"

//Data for connection to the "Firebase" database
#define FIREBASE_HOST "https://lawnmower-android-66b33.firebaseio.com"
#define FIREBASE_AUTH "4YOEWZHxqEOpZokJi5bhdcMpuIy15Qwgh45MKTO2"


//Define FirebaseESP8266 data object for data sending and receiving
FirebaseData fbdo;

//Variables
String receiveCar; // Car Received Data
String sendReceive; // Selection of active Tasks
String firebaseReceive; // Data Received from Firebase
String strJsonFirebase; // Data Json Received from Firebase
String strJsonClient = "{\"battery\":\" \",\"start_status\":\" \",\"latitude\":\" \",\"longitude\":\" \",}"; // Data Json Received from client

String clientInput; // Car Received Data
String latitude;  // Car Received Data
String longitude; // Car Received Data
String battery; // Received from the car

String start_status; // Received from Firebase
String _0_latitude;  // Received from Firebase
String _0_longitude; // Received from Firebase
String _1_latitude;  // Received from Firebase
String _1_longitude; // Received from Firebase
String _2_latitude;  // Received from Firebase
String _2_longitude; // Received from Firebase
String _3_latitude;  // Received from Firebase
String _3_longitude; // Received from Firebase


// Semaphore allows access to variables by different tasks at different times
SemaphoreHandle_t myMutex;

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
// Method for read String containing JSON
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

// ------------------------------------------- //------------------------------------------//
// Method write Json
String writeJson(String strJsonInput, String indexJsonInput, String insertJson) {
  String strJson = strJsonInput;
  int lengthstrJson = strJson.length();
  String indexJson = indexJsonInput;
  int lengthIndexJson = indexJson.length();
  String strInsertJson = insertJson;
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
  String strInicio = strJson.substring(0, cont + lengthIndexJson + 3);
  while (positionWord == "n") {
    if (!strJson.startsWith("\"", (cont + indexJson.length() + 3 + cont1))) {
      cont1++;
    } else {
      positionWord = "s";
      break;
    }
  }
  String strFinal = strJson.substring(cont + 3 + cont1 + lengthIndexJson, lengthstrJson);
  if (readValue == 's') {
    return valueJson = strInicio + strInsertJson + strFinal;
  } else {
    return "false";
  }
}

//-------------------------------------------- // -----------------------------------------//


void setup() {
  Serial.begin(115200);

  // Start variable:
  strJsonFirebase = "{\"battery\":\" \",\"start_status\":\" \",";
  strJsonFirebase += "\"_0_latitude\":\" \",\"_0_longitude\":\" \",\"_1_latitude\":\" \",\"_1_longitude\":\" \",";
  strJsonFirebase += "\"_2_latitude\":\" \",\"_2_longitude\":\" \",\"_3_latitude\":\" \",\"_3_longitude\":\" \",";

  // Configuration of the .TXT File - Data:
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Creation of files that inform if you are sending or receiving data:
  createFiles("sendReceive", "0");
  createFiles("firebaseReceive", strJsonFirebase);
  createFiles("strJsonFirebase", strJsonFirebase);
  createFiles("clientInput", "0");

  /*Initializes Variables:*/
  sendReceive = setStatusVar("sendReceive");
  firebaseReceive = setStatusVar("firebaseReceive");
  strJsonFirebase = setStatusVar("strJsonFirebase");
  clientInput = setStatusVar("clientInput");

  /*Update variables reveive from firebase*/
  start_status =  readJson(strJsonFirebase, "start_status"); // Received from Firebase
  _0_latitude = readJson(strJsonFirebase, "_0_latitude");  // Received from Firebase
  _0_longitude = readJson(strJsonFirebase, "_0_longitude"); // Received from Firebase
  _1_latitude = readJson(strJsonFirebase, "_1_latitude");  // Received from Firebase
  _1_longitude = readJson(strJsonFirebase, "_1_longitude"); // Received from Firebase
  _2_latitude = readJson(strJsonFirebase, "_2_latitude");  // Received from Firebase
  _2_longitude = readJson(strJsonFirebase, "_2_longitude"); // Received from Firebase
  _3_latitude = readJson(strJsonFirebase, "_3_latitude");  // Received from Firebase
  _3_longitude = readJson(strJsonFirebase, "_3_longitude"); // Received from Firebase


  // Test
  Serial.println("strJsonFirebase: " + strJsonFirebase);

  // Acces Point and SoftAP configuration:
  WiFi.mode(WIFI_AP_STA); //Access Point mode
  WiFi.softAP(ssidEspServer, PassEspServer);    //Password length minimum 8 char

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  //connect to WiFi
  WiFi.begin(ssidWiFiLocalSet, passwordSenhaWiFiLocal);

  byte contWiFi = 0;
  while ((WiFi.status() != WL_CONNECTED) and contWiFi < 30) {
    delay(500);
    Serial.print(".");
    contWiFi++;
  }

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.println("[WIFI] Connecting");
  delay(500);
  Serial.println(WiFi.localIP());

  //Set your Firebase info
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  //Enable auto reconnect the WiFi when connection lost
  Firebase.reconnectWiFi(true);

  //Start Server
  server.begin();

  // Semaphore and Tasks creation
  myMutex = xSemaphoreCreateMutex();
  if (myMutex != NULL) {
    //create a task
    xTaskCreatePinnedToCore(
      TaskWiFi,   /* Task function. */
      "TaskWiFi",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      3,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
    delay(500);

    //create a task
    xTaskCreatePinnedToCore(
      TaskLocalControl,   /* Task function. */
      "TaskLocalControl",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      3,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */
    delay(500);

    //create a task
    xTaskCreatePinnedToCore(
      TaskRemoteControl,   /* Task function. */
      "TaskRemoteControl",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      4,           /* priority of the task */
      NULL,      /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */
    delay(500);

  }
}

// Task WiFi
void TaskWiFi( void * pvParameters ) {
  for (;;) {
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    if (WiFi.status () == WL_CONNECTED) {
      printGlobal("wificonnect !!!!");
    }
    else {
      printGlobal("falhou !!!!");
      WiFi.reconnect ();
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}


// Task TaskLocalControl
void TaskLocalControl( void * pvParameters ) {
  for (;;) {
    /*Initializes the Variables:*/
    printGlobal("Local-Control");
    xSemaphoreTake(myMutex, portMAX_DELAY);
    sendReceive = setStatusVar("sendReceive");

    xSemaphoreGive(myMutex);

    if (sendReceive.toInt() == 0) { // Connection Selection
      WiFiClient client = server.available();   // Listen for incoming clients
      String header;
      char c;
      String OpcaoAcao;
      if (client) {                             // If a new client connects,
        ////Serial.println("New Client.");          // print a message out in the //Serial port
        String currentLine = "";                // make a String to hold incoming data from the client
        if (client.connected()) {            // loop while the client's connected
          //if (client.connected()) {
          while (client.available()) {             // if there's bytes to read from the client,
            c = client.read();             // read a byte, then
            ////Serial.write(c);                    // print it out the //Serial monitor
            header += c;
            //Serial.print(header);
            if ((c == '\n') and (header.length() > 49)) {

              // Save the last data received from the slave in a file
              xSemaphoreTake(myMutex, portMAX_DELAY);
              setStatusFiles("clientInput", header);
              xSemaphoreGive(myMutex);

            }

          }

          // Send the last data received from Firebase to the slave
          client.print("{\"start_status\":\" " + start_status + " \",");
          client.print("\"_0_latitude\":\" " + _0_latitude + " \",");
          client.print("\"_0_longitude\":\" " + _0_longitude + " \",");
          client.print("\"_1_latitude\":\" " + _1_latitude + " \",");
          client.print("\"_1_longitude\":\" " + _1_longitude + " \",");
          client.print("\"_2_latitude\":\" " + _2_latitude + " \",");
          client.print("\"_2_longitude\":\" " + _2_longitude + " \",");
          client.print("\"_3_latitude\":\" " + _3_latitude + " \",");
          client.print("\"_3_longitude\":\" " + _3_longitude + " \",");
          client.flush();
          client.stop();
        }
      }
      if (header.length() > 10) {

        // Save the last data received from Slave
        xSemaphoreTake(myMutex, portMAX_DELAY);
        start_status = readJson(header, "start_status"); //Test
        latitude = readJson(header, "latitude"); //Test
        longitude = readJson(header, "longitude"); //Test
        battery = readJson(header, "battery"); //Test
        clientInput = header;
        xSemaphoreGive(myMutex);

        printGlobal(battery);//Test
        printGlobal(receiveCar);//Test
      }
      header = "";
      setStatusFiles("sendReceive", "1");
    }
    vTaskDelay(110 / portTICK_PERIOD_MS); // original valor: 110
  }
}

// Task TaskRemoteControl - Send and Receive Data from Firebase
void TaskRemoteControl( void * pvParameters ) {
  for (;;) {
    if (sendReceive.toInt() == 1) {
      // Send Firebase
      xSemaphoreTake(myMutex, portMAX_DELAY);
      if (Firebase.setString(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/battery", battery)) 
      {
        //Success
        Serial.println("Set string data success");
      } else {
        //Failed?, get the error reason from fbdo
        Serial.print("Error string setString, ");
        Serial.println(fbdo.errorReason());
      }

      // Get Firebase
      xSemaphoreGive(myMutex);

      // Get Firebase /cfp1Uk5Vykbdgh44KVlkyFCyE4c2/start_status
      if (Firebase.getBool(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/start_status"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        start_status = String(fbdo.boolData());
        strJsonFirebase = writeJson(strJsonFirebase, "start_status", start_status);
        xSemaphoreGive(myMutex);
        printGlobal("start_status: ");
        printGlobal(start_status);
      }
      else {
        //Failed?, get the error reason from fbdo
        Serial.print("Error getting start status, ");
        Serial.println(fbdo.errorReason());
      }

      // Get Firebase custom_lawns/test1/locations/mapPositions/0/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/0/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _0_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_0_latitude", _0_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_0_latitude: ");
        printGlobal(_0_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/0/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _0_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_0_longitude", _0_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_0_longitude: ");
        printGlobal(_0_longitude);
      }
      // Get Firebase custom_lawns/test1/locations/mapPositions/1/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/1/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _1_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_1_latitude", _1_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_1_latitude: ");
        printGlobal(_1_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/1/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _1_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_1_longitude", _1_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_1_longitude: ");
        printGlobal(_1_longitude);
      }
      // Get Firebase custom_lawns/test1/locations/mapPositions/2/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/2/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _2_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_2_latitude", _2_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_2_latitude: ");
        printGlobal(_2_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/2/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _2_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_2_longitude", _2_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_2_longitude: ");
        printGlobal(_2_longitude);
      }
      // Get Firebase custom_lawns/test1/locations/mapPositions/3/
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/3/latitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _3_latitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_3_latitude", _3_latitude);
        xSemaphoreGive(myMutex);
        printGlobal("_3_latitude: ");
        printGlobal(_3_latitude);
      }
      if (Firebase.getDouble(fbdo, "/cfp1Uk5Vykbdgh44KVlkyFCyE4c2/default_lawn/locations/positions/0/longitude"))
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        _3_longitude = String(fbdo.doubleData(), 6);
        strJsonFirebase = writeJson(strJsonFirebase, "_3_longitude", _3_longitude);
        xSemaphoreGive(myMutex);
        printGlobal("_3_longitude: ");
        printGlobal(_3_longitude);
      }


      // Save data receive from firebase
      setStatusFiles("strJsonFirebase", strJsonFirebase);
      printGlobal(strJsonFirebase);
      xSemaphoreTake(myMutex, portMAX_DELAY);
      setStatusFiles("sendReceive", "0");
      xSemaphoreGive(myMutex);
    }
    //vTaskDelay(110 / portTICK_PERIOD_MS);
    vTaskDelay(110 / portTICK_PERIOD_MS);
  }
}

void loop() {
}
