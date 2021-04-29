#include <WiFi.h>
#include <WiFiMulti.h>

//Txt files
#include <SPIFFS.h>
#include "FS.h"

WiFiMulti WiFiMulti;
String receiveData;
String battery ; // Percentage value of battery
String start_status; // Car status
String latitude; // Current car latitude
String longitude; // Current car longitude
String _0_latitude; // Orientation coordinates
String _0_longitude; // Orientation coordinates
String _1_latitude; // Orientation coordinates
String _1_longitude; // Orientation coordinates
String _2_latitude; // Orientation coordinates
String _2_longitude; // Orientation coordinates
String _3_latitude; // Orientation coordinates
String _3_longitude; // Orientation coordinates

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
}// Final Method

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

void setup()
{
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
  
  // Semaphore to allow multipoint access to variables without conflict
  myMutex = xSemaphoreCreateMutex();
}


void loop()
{
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
  client.print("[{\"battery\":\"" + battery + "\",\"latitude\":\"" + latitude + "\",\"longitude\":\"" + longitude + "\"}]");

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
      // Save variables received from the "Esp32 Master"
      start_status = readJson(receiveData, "start_status"); //Test
      _0_latitude = readJson(receiveData, "_0_latitude"); //Test
      _0_longitude = readJson(receiveData, "_0_longitude"); //Test
      _1_latitude = readJson(receiveData, "_1_latitude"); //Test
      _1_longitude = readJson(receiveData, "_1_longitude"); //Test
      _2_latitude = readJson(receiveData, "_2_latitude"); //Test
      _2_longitude = readJson(receiveData, "_2_longitude"); //Test
      _3_latitude = readJson(receiveData, "_3_latitude"); //Test
      _3_longitude = readJson(receiveData, "_3_longitude"); //Test


      // Test the variables
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
}
