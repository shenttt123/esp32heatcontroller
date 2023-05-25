/*

  Reads thermocouple temperatures from MCP9600 sensors.
  The MCP9600 supports up to five K/J/T/N/S/E/B/R types of thermocouples, and the global type can be set on sc card.
  Performs PI control for up to 5 temperature loops (heaters). Number of loops and global set-point range can be configured on sd card.
  Controls CleO display.
  Performs PI auto tune.
  Provides WiFi access point for loop configuration. WebSockets are used for device to browser communication.

  Hardware Connections:
  Attach the Shield to the ESP32. Shield provides 5 thermocouple connections and 4 digital outputs for heater control. The 1st output is on the power board.
  Serial.print at 115200 baud to serial monitor. if DEBUG = 1.
  
  11/11/2021: Increase Label length to 9 characters, display PV and %Out with only 1 digit after decimal point.
        Add checkboxes to enable/disable access to setpoint changes and autotune.
        Add loop runaway detection. Blink loop row if error detected and lock out output.
        Add global min/max for setpoint limits and thermocouple type (sd card only).
        Add sd card file for number of loops.
  11/15/2021: Show Alarms in autotune dialog.
        Replace blocking "Alert" in web page.
        Update Kp/Ki in Setup web page after "save" in autotune.
  11/17/2021: Turn On buzzer if temperature or loop alarm.
  11/18/2021: Detect open "Minus" thermocouple wire.
  12/15/2021: Limit autotune step output range. Put PID in MANUAL mode during tuning...
  01/13/2021: Add keypad button and code to turn loop output ON/OFF. Fix Sparkfun MCP9600 library (set filter).
  01/19/2022: Move current loop init code right after I2C init to limit initial buzzer sound.
  01/20/2022: Set temp[loop] to zero if sensor not available to detect disconnected TC after boot-up. Fix LOOP ALARM reset.
  01/24/2022: Do not create alarm when output is disabled but show high and low temps via label color change.
  03/30/2022: Add SP to auto tune dialog. Change tc temp timeout code.
  03/30/2022: Display LERR for output% if loop error
  04/04/2022: Add Temperature Offset (txt files).
  04/20/2022: Put PID to manual mode in autotune stabalizing step.
  04/20/2022: Default output to OFF after startup. Add delay after ENTER button touch.
  05/10/2022: Change LoopError algorithm.
  11/02/2022: Make consecutice Loor Errors a variable on SD card.
  11/09/2022: Increase sensor read intervall from 150 msec to 200 msec
  03/15/2013: Add ability to reset Errors by turning loop OFF -> ON
*/

// Network Libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// SPI Flash File System Library
#include "SPIFFS.h"

// Display Library
#include <CleO.h>
// PID Library
#include <PID_v1.h>
// GPIO Expander Library
#include <SparkFun_PCA9536_Arduino_Library.h>
PCA9536 io;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define DEBUG 0                // Set to "1" to enable Serial.print

String APname = "";
const char* password = "ashbycrosshc";
String TitleStr = "BullsEye Heater Controller";
String VersionStr = "Version 1.62 OTA";

// Buffers
char buf[128];
// Landscape Format
int screenWidth = 480;
int screenheight = 320;
// File handle
int16_t file;
int16_t logo;

// Thermocouple MCP9600 library. Modified to detecet TC open circuit and short.
#include <SparkFun_MCP9600.h>

MCP9600 tempSensor[] = {{}, {}, {}, {}, {}};

float temp[5];
float temp_old[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float temp_prev[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float tempOffset[] = {0.0, 0.0, 0.0, 0.0, 0.0};
bool sensor_online[5];

uint8_t coefficient = 5; // 0 .. 7

Thermocouple_Type type = TYPE_J; //the type of thermocouple to change to! (K/J/T/N/S/E/B/R = 0,1,2,3,4,5,6,7,)
String tcChar[] = {"K", "J", "T", "N", "S", "E", "B", "R"};

Ambient_Resolution ambientRes = RES_ZERO_POINT_0625;
Thermocouple_Resolution thermocoupleRes = RES_14_BIT;

// PID
int numLoops = 5;
double SpMin = 60.0;
double SpMax = 200.0;
double Setpoint[] = {140.0, 140.0, 120.0, 100.0, 90.0};
double Input[] = {60.0, 60.0, 60.0, 60.0, 60.0};
double Output[] = {0.0, 0.0, 0.0, 0.0, 0.0};
double Kp[] = {50.0, 50.0, 50.0, 50.0, 50.0};
double Ki[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float outPercent[] = {0.0, 0.0, 0.0, 0.0, 0.0};
double lowAlarm[] = {0.0, 0.0, 0.0, 0.0, 0.0};
double highAlarm[] = {0.0, 0.0, 0.0, 0.0, 0.0};
int AlarmStatus[] = {0, 0, 0, 0, 0};                 // 0=NO_ALARM; 1=LOW_ALARM; 2=HIGH_ALARM
int LoopError[] = {0, 0, 0, 0, 0};                   // 0=OK; 1=Loop Error; 2=Open or Short circuit
int LoopErrCtr[] = {0, 0, 0, 0 ,0};
//int LoopErrCtr_Old[] = {0, 0, 0, 0 ,0};
int tmoutsec = 300;
int sumErr = 10;

unsigned long time_equal[] = {0, 0, 0, 0 ,0};
unsigned long time_unequal[] = {0, 0, 0, 0 ,0};

unsigned long windowStartTime[5];
int WindowSize = 2000;
double KpTuning = 100.0;
double deltaOut = 0.6;
bool tune[] = {false, false, false, false, false};
bool tune_old[] = {false, false, false, false, false};
bool atInit[] = {false, false, false, false, false};
int cycle = 0;
int loopNum = 0;
double AT_Kp = 0.0;
double AT_Ki = 0.0;

bool save_loop_data = false;
bool save_tuning_data = false;
int loopStatus[] = {0, 0, 0, 0, 0}; // 0=OFF; 1=AUTO; 2=Stabalizing; 3=AutoTune
float ambientTemp[] = {50.0, 50.0, 50.0, 50.0, 50.0};
int loopCtr = 0;
int OKloopCtr = 0;

bool outputEnable[] = {false, false, false, false, false};

// Create array of PID objects
PID pidLoop[] = {
  PID(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], 0.0, DIRECT),
  PID(&Input[1], &Output[1], &Setpoint[1], Kp[1], Ki[1], 0.0, DIRECT),
  PID(&Input[2], &Output[2], &Setpoint[2], Kp[2], Ki[2], 0.0, DIRECT),
  PID(&Input[3], &Output[3], &Setpoint[3], Kp[3], Ki[3], 0.0, DIRECT),
  PID(&Input[4], &Output[4], &Setpoint[4], Kp[4], Ki[4], 0.0, DIRECT)
};

// GUI Button States
int16_t oldtag = 0;
boolean overviewDisplay = true;
boolean keypadDisplay = false;
boolean autotuneDisplay = false;
boolean isOverviewDisplay = false;
boolean isKeypadDisplay = false;
boolean isAtDisplay = false;
boolean newEntry = false;
String targetStr[] = {"", "", "", "", ""};
String loopLabel[] = {"Loop 0", "Loop 1", "Loop 2", "Loop 3", "Loop 4"};
String Title1 = "";
String Title2 = "";
String AlarmText = "";
String LoopAlarmText = "";
String StatusStr1 = "";
String StatusStr2 = "";
String spEnableStr = "1";
String atEnableStr = "1";

boolean AlarmBlink = false;

boolean ws_connected = false;

int loopNumber = 0;
int AT_loopNum = 0;

// Keypad
#define BTN_WIDTH 80
#define BTN_HEIGHT 50
#define XGAP 10

// Digital Output Loop Pins
#define LOOP0_OUT_PIN 13
#define LOOP1_OUT_PIN 0
#define LOOP2_OUT_PIN 1
#define LOOP3_OUT_PIN 2
#define LOOP4_OUT_PIN 3

// Digital Input Pins
#define INPUT_PIN 21

// Digital Output Pins
#define RESET_PIN 15

//This is the I2C Address of the MCP4725 DAC, by default (A0 pin on chip pulled to GND).
#define MCP4725_ADDR 0x60


void setup() {

  if (DEBUG == 1) Serial.begin(115200);

  delay(100);

  // Initialize SPIFFS (File System)
  if(!SPIFFS.begin(true)){
    if (DEBUG == 1) Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  initWiFiAP();

  // On HTTP request for root, provide index.html file
  server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP request for style sheet, provide styles.css
  server.on("/styles.css", HTTP_GET, onCSSRequest);

  // On HTTP request for javascript, provide script.js
  server.on("/script.js", HTTP_GET, onJsRequest);
  
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/favicon.ico", "image/x-icon");
  });
  
  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Start web server
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  for (int i = 0; i < numLoops; i++) {
    sensor_online[i] = false;
  }
  
  Wire.begin();
  delay(100);
  Wire.setClock(10000);
  CurrentLoop(0); // 0..20 mA --> Turn Buzzer OFF
  delay(1000);

  for (int j = 0; j < numLoops; j++){
    tempSensor[j].begin(97 + j); // 1st sensor address (0x61; 97dec) for Thermocouple Amplifier
    //check that the sensor is connected
    if (tempSensor[j].isConnected()) {
      if (DEBUG == 1) {Serial.print("Sensor "); Serial.print(j); Serial.println( " OK!");}
	      tempSensor[j].resetToDefaults();
		  delay(10);
          tempSensor[j].setAmbientResolution(ambientRes);
		  delay(10);
          tempSensor[j].setThermocoupleResolution(thermocoupleRes);
          sensor_online[j] = true;
    }
    // Configure PID
    Input[j] = Setpoint[j];
    windowStartTime[j] = millis();
  }

  CleO.begin();
  CleO.DisplayRotate(2, 0); // Landscape Mode
  readLogo();
  readStartupData();
  setFilter();
  setType();
  displayStartupScreen();
  delay(7000);
  
  for (int i = 0; i < numLoops; i++) {
    pidLoop[i].SetOutputLimits(0, WindowSize);
    pidLoop[i].SetMode(AUTOMATIC);
    pidLoop[i].SetSampleTime(1000);
    pidLoop[i].SetTunings(Kp[i], Ki[i], 0.0);
    loopStatus[i] = 1;
  }

  pinMode(LOOP0_OUT_PIN, OUTPUT);
  digitalWrite(LOOP0_OUT_PIN, LOW);
  
  // Initialize the PCA9536 (I2C GPIO expander on Shot Controller 6 IO board))
  if (io.begin() == false)
  {
    if (DEBUG == 1) Serial.println("PCA9536 not detected. Please check wiring. Freezing...");
    while (1);
  }
  io.pinMode(LOOP1_OUT_PIN, OUTPUT);
  io.pinMode(LOOP2_OUT_PIN, OUTPUT);
  io.pinMode(LOOP3_OUT_PIN, OUTPUT);
  io.pinMode(LOOP4_OUT_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT);

  io.digitalWrite(LOOP1_OUT_PIN, LOW);
  io.digitalWrite(LOOP2_OUT_PIN, LOW);
  io.digitalWrite(LOOP3_OUT_PIN, LOW);
  io.digitalWrite(LOOP4_OUT_PIN, LOW);

}

void loop() {

  // read the thermocouple temperatures
  if (runEvery_0(200)) {
      if (sensor_online[loopCtr]) {
          //ambientTemp[loopCtr] = tempSensor[loopCtr].getAmbientTemp(false);
          //if (DEBUG == 1) {Serial.print(loopCtr); Serial.print(": "); Serial.println(ambientTemp[loopCtr]);}
		  //Serial.print(loopCtr); Serial.print(" : "); Serial.println(tempSensor[loopCtr].getStatus());
          if (tempSensor[loopCtr].available()) {
              temp[loopCtr] = tempSensor[loopCtr].getThermocoupleTemp(false); // False -> Fahrenheit; True -> Celsius
              temp[loopCtr] = temp[loopCtr] + tempOffset[loopCtr];
			  //float filterFactor = 1.0 - (float(coefficient) / 10.0);
              //temp[loopCtr] = filterFactor * temp[loopCtr] + (1.0 - filterFactor) * temp_prev[loopCtr];
			  //temp_prev[loopCtr] = temp[loopCtr];
			  OKloopCtr = loopCtr;
          }
          else {
              //temp[loopCtr] = 0.0; // add counter array, increment counter[], if counter[] > 10, set temp[] to zero and reset counter
              LoopErrCtr[loopCtr] = LoopErrCtr[loopCtr] + 1;
          }
          Input[loopCtr] = temp[loopCtr];

          // Detect Temperature Alarms
          if ((temp[loopCtr] >= highAlarm[loopCtr]) && (highAlarm[loopCtr] > 0.5)) {
              AlarmStatus[loopCtr] = 2;
          }
          else if ((temp[loopCtr] <= lowAlarm[loopCtr]) && (lowAlarm[loopCtr] > 0.5)) {
              AlarmStatus[loopCtr] = 1;
          }
          else {
              AlarmStatus[loopCtr] = 0;
          }

          if (temp[loopCtr] < 0.1 && outputEnable[loopCtr]) {
              LoopError[loopCtr] = 2;
              LoopAlarmText = "LOOP ALARM";
          }
      }
      loopCtr = loopCtr + 1;
      if (loopCtr >= numLoops) loopCtr = 0;

      // Create Alarm Text for Status Line
      AlarmText = "";
      for (int i = 0; i < numLoops; i++) {
          if (AlarmStatus[i] > 0 && outputEnable[i]) {
              AlarmText = "TEMP ALARM";
          }
      }
  }
  // Check if thermocouple short circuit or heater not connected when output enabled
  if (delayed_runEvery(2000)) {
      for (int i = 0; i < numLoops; i++) {
          int tempInt = int(temp[i] * 10.0);
          int tempInt_old = int(temp_old[i] * 10.0);
          if (tempInt != tempInt_old) {
              time_unequal[i] = millis();
          }
          else {
              time_equal[i] = millis();
          }
          if (outPercent[i] > 99.9 && outputEnable[i] && time_equal[i] > time_unequal[i] && temp[i] > 0.0) {
              unsigned long time_delta = time_equal[i] - time_unequal[i];
              if (time_delta >= tmoutsec * 1000UL) {
                  LoopError[i] = 1;
                  LoopAlarmText = "LOOP ALARM";
                  if (DEBUG == 1) {Serial.print("LoopAlarm "); Serial.println(i);}
              }
          }
          temp_old[i] = temp[i];
     }
  }

  // Send data to browser (ws)
  if (runEvery_2(2000) && ws_connected) {
      // SP
      String wsTxt = "data:," + String(Setpoint[loopNumber]) + ",";
      // PV
      wsTxt += String(temp[loopNumber]) + ",";
      // OUTPUT
      if (outputEnable[loopNumber]) {
          wsTxt += String(outPercent[loopNumber]) + ",";
      }
      else {
          wsTxt += "OFF,";
      }
      // PID Setup
      wsTxt += String(pidLoop[loopNumber].GetKp()) + "," + String(pidLoop[loopNumber].GetKi()) + ",";
      // Loop Status
      wsTxt += String(loopStatus[loopNumber]) + ",";
      // Cycle
      wsTxt += String(cycle) + ",";
      // Send it
      wsTxt.toCharArray(buf, 60);
      ws.textAll(buf);
  }

  // Blink Alarm generation, alarm buzzer control
  if (runEvery_3(500)) {
    AlarmBlink = !AlarmBlink;
    int buzzer = 0;
    for (int i = 0; i < numLoops; i++) {
        if (outputEnable[i] && sensor_online[i] && (LoopError[i] > 0 || AlarmStatus[i] > 0)) {buzzer = 1000;} // 0..20 mA --> Turn Buzzer ON
    }
	CurrentLoop(buzzer);
  }

  // Check if Thermocouple Error Count exceeds limit
  if (runEvery_4(numLoops * 220 * sumErr)) {
    for (int i = 0; i < numLoops; i++) {
       if (LoopErrCtr[i] >= sumErr) temp[i] = 0.0;
       LoopErrCtr[i] = 0;
	}
  }

  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  for (int i = 0; i < numLoops; i++){
    if (sensor_online[i]) {
        pidLoop[i].Compute();
        if (tune[i]) {
            autotune(AT_loopNum);
        }
    }
  }

  for (int i = 0; i < numLoops; i++) {
    unsigned long now = millis();
    if (now - windowStartTime[i] > WindowSize) {
      //time to shift the Relay Window
      windowStartTime[i] += WindowSize;
    }
    if ((Output[i] > now - windowStartTime[i]) && (LoopError[i] == 0) && (outputEnable[i])) {
      if (i == 0) digitalWrite(LOOP0_OUT_PIN, HIGH);
      if (i == 1) io.digitalWrite(LOOP1_OUT_PIN, HIGH);
      if (i == 2) io.digitalWrite(LOOP2_OUT_PIN, HIGH);
      if (i == 3) io.digitalWrite(LOOP3_OUT_PIN, HIGH);
      if (i == 4) io.digitalWrite(LOOP4_OUT_PIN, HIGH);
    }
    else {
      if (i == 0) digitalWrite(LOOP0_OUT_PIN, LOW);
      if (i == 1) io.digitalWrite(LOOP1_OUT_PIN, LOW);
      if (i == 2) io.digitalWrite(LOOP2_OUT_PIN, LOW);
      if (i == 3) io.digitalWrite(LOOP3_OUT_PIN, LOW);
      if (i == 4) io.digitalWrite(LOOP4_OUT_PIN, LOW);
    }
    outPercent[i] = Output[i] * 100.0 / WindowSize; // For Display on HMI (output in %)
  }


  CleO.Start();
  //if (overviewDisplay) {
    display();
  //}
  if (keypadDisplay) {
    displayKeypad();
  }
  if (autotuneDisplay) {
    displayAtDialog();
  }
  control();
  if (save_loop_data) {
    save_loop_data = false;
    String wsTxt = "saved:";
    wsTxt.toCharArray(buf, 60);
    ws.textAll(buf);
    saveLoopDataToSD(loopNumber);
  }
  if (save_tuning_data) {
    save_tuning_data = false;
    String wsTxt = "saved:";
    wsTxt.toCharArray(buf, 60);
    ws.textAll(buf);
    saveTuningDataToSD();
  }
  CleO.Show();

}

/*
  Function allows to execute code very x milliseconds: e.g. if (runEvery(1000)) {}
*/
boolean runEvery_0(unsigned long interval) {
  static unsigned long previousMillis_0 = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_0 >= interval)
  {
    previousMillis_0 = currentMillis;
    return true;
  }
  return false;
}

boolean delayed_runEvery(unsigned long interval) {
  static unsigned long previousMillis_1 = millis();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_1 >= interval)
  {
    previousMillis_1 = currentMillis;
    return true;
  }
  return false;
}

boolean runEvery_2(unsigned long interval) {
  static unsigned long previousMillis_2 = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_2 >= interval)
  {
    previousMillis_2 = currentMillis;
    return true;
  }
  return false;
}

boolean runEvery_3(unsigned long interval) {
  static unsigned long previousMillis_3 = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_3 >= interval)
  {
    previousMillis_3 = currentMillis;
    return true;
  }
  return false;
}

boolean runEvery_4(unsigned long interval) {
  static unsigned long previousMillis_4 = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_4 >= interval)
  {
    previousMillis_4 = currentMillis;
    return true;
  }
  return false;
}

/*
Function reads Startup Data from SD Card
*/
void readLogo() {
  /* Get the handle by loading bitmap */
  logo = CleO.LoadImageFile("AshbyCross/ashby_logo.jpg", 0);
  CleO.RectangleJustification(TL);
}


/*
Function displays Startup Screen
*/
void displayStartupScreen() {
    char apchars[64];
    CleO.Start();
    TitleStr.toCharArray(apchars, TitleStr.length()+1);
    CleO.StringExt(FONT_SANS_3, 50, 20, ORANGE, ML, 0, 0, apchars);
    VersionStr.toCharArray(apchars, VersionStr.length()+1);
    CleO.StringExt(FONT_SANS_3, 50, 80, LIGHTSKYBLUE, ML, 0, 0, apchars);
    CleO.StringExt(FONT_SANS_3, 50, 110, LIGHTGREY, ML, 0, 0, "Connect to ACCESS POINT");
    APname.toCharArray(apchars, APname.length()+1);
    CleO.StringExt(FONT_SANS_3, 50, 140, LIGHTGREY, ML, 0, 0, apchars);
    CleO.StringExt(FONT_SANS_3, 50, 170, LIGHTGREY, ML, 0, 0, "IP Address: 192.168.4.1");
    StatusStr1.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_3, 50, 220, LIGHTGREY, ML, 0, 0, buf);
    StatusStr2.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_3, 50, 250, LIGHTGREY, ML, 0, 0, buf);
    CleO.Show();
}

/*
Function Displays Main Page
*/
void display() {
    isKeypadDisplay = false;
    isAtDisplay = false;
    // Draw Logo
    CleO.Bitmap(logo, 0, 0);  // Display Logo
    String labelText = "";
    String pvText = "";
    String spText = "";
    String outText = "";
    
    // Draw Title Headers
	Title1 = "Heater Controller";
	Title2 = "PID Loop Overview";
    Title1.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_4, 98, 0, LIGHT_BLUE, TL, 0, 0, buf);
    Title2.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_4, 98, 35, LIGHT_BLUE, TL, 0, 0, buf);

    // Draw horizontal Line
    CleO.LineColor(LIGHT_GREY);
    CleO.Line(0, 78, screenWidth, 78);

    for (int i = 0; i < numLoops ; i++) {
      if (sensor_online[i]) {
        labelText = loopLabel[i];
        spText = String(int(round(Setpoint[i]))) + " F";
        if (outputEnable[i] && LoopError[i] == 0) {
            outText = String(dtostrf(outPercent[i], 1, 1, buf)) + "%"; // One digit after decimal point
        }
        else if (outputEnable[i] && LoopError[i] == 1) {
            outText = "ERR1";
        }
        else if (outputEnable[i] && LoopError[i] == 2) {
            outText = "ERR2";
        }
        else {
            outText = "OFF";
        }
        if (sensor_online[i]) {
            pvText = String(dtostrf(temp[i], 1, 1, buf)) + " F"; // One digit after decimal point
        }
        else {
          pvText = "N/A";
        }
        labelText.toCharArray(buf, 60);
        CleO.Tag(i * 4 + 20); // 20 .. 36
        if ((AlarmStatus[i] == 0 && LoopError[i] == 0) || (AlarmStatus[i] == 0 && LoopError[i] > 0 && AlarmBlink)) {
          CleO.StringExt(FONT_SANS_4, 5, 105 + (i * 38), LIGHT_BLUE, ML, 0, 0, buf);
        }
        else if ((AlarmStatus[i] == 1 && LoopError[i] == 0) || (AlarmStatus[i] == 1 && LoopError[i] > 0 && AlarmBlink)) {
          CleO.StringExt(FONT_SANS_4, 5, 105 + (i * 38), BLUE, ML, 0, 0, buf);
        }
        else if ((AlarmStatus[i] == 2 && LoopError[i] == 0) || (AlarmStatus[i] == 2 && LoopError[i] > 0 && AlarmBlink)) {
          CleO.StringExt(FONT_SANS_4, 5, 105 + (i * 38), RED, ML, 0, 0, buf);
        }

        pvText.toCharArray(buf, 60);
        CleO.Tag(i * 4 + 21); // 21 .. 37
        if (LoopError[i] == 0 || (LoopError[i] > 0 && AlarmBlink)) CleO.StringExt(FONT_SANS_4, 220, 105 + (i * 38), RED, MM, 0, 0, buf);
        spText.toCharArray(buf, 60);
        if (spEnableStr == "1") CleO.Tag(i * 4 + 22); // 22 .. 38
        if (LoopError[i] == 0 || (LoopError[i] > 0 && AlarmBlink)) CleO.StringExt(FONT_SANS_4, 320, 105 + (i * 38), GREEN, MM, 0, 0, buf);
        outText.toCharArray(buf, 60);
        if (atEnableStr == "1") CleO.Tag(i * 4 + 23); // 23 .. 39
        if (LoopError[i] == 0 || (LoopError[i] > 0 && AlarmBlink)) CleO.StringExt(FONT_SANS_4, 430, 105 + (i * 38), ORANGE, MM, 0, 0, buf);
      }
    }

    // Draw horizontal line
    CleO.Tag(99);
    CleO.Line(0, 280, screenWidth, 280);
    // Display Alarms
    String StatusText = AlarmText + "   " + LoopAlarmText;
    StatusText.toCharArray(buf, 60);
    if (AlarmBlink) CleO.StringExt(FONT_SANS_4, screenWidth/2, 300, RED, MM, 0, 0, buf);
    String loopCtrStr = String(OKloopCtr);
    loopCtrStr.toCharArray(buf, 10);
    if (OKloopCtr != 9) {CleO.StringExt(FONT_SANS_4, 2, 300, DARKSLATEGRAY, ML, 0, 0, buf);}
    else {CleO.StringExt(FONT_SANS_4, 2, 300, DARKSLATEGRAY, ML, 0, 0, " ");}
    OKloopCtr = 9;

    isOverviewDisplay = true;
}

/*
Function Displays Keypad to change set-points
*/
void displayKeypad() {
    isAtDisplay = false;
    isOverviewDisplay = false;

    String labelText = "";
    CleO.RectangleCorner(1);
    CleO.RectangleColor(BLACK);
    CleO.RectangleXY(0, 78, 479, 240);
    // Button Color
    CleO.eve_fgcolor(YELLOW);
    byte xk;
    byte yk;

   // Clear Header Window
    CleO.RectangleCorner(1);
    CleO.RectangleColor(BLACK);
    CleO.RectangleXY(80, 0, 400, 80);

    labelText = loopLabel[loopNum] + ": " + targetStr[loopNum] + " F";
    labelText.toCharArray(buf, 50);
    CleO.StringExt(FONT_SANS_4, 80, 20, GREEN, TL, 0, 0, buf);
    CleO.StringExt(FONT_SANS_4, 400, 20, BLACK, TL, 0, 0, "..");

    // Draw keys 1..9
    for (xk = 1; xk <= 3; xk++){
        for (yk = 1; yk <= 3; yk++){
           String displayText = String(xk + (yk-1) * 3);
           displayText.toCharArray(buf, 4);
           CleO.Tag(xk + (yk-1) * 3);
           CleO.eve_button(xk * (BTN_WIDTH + XGAP) - BTN_WIDTH, (yk * 60) + 20, BTN_WIDTH, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, buf);
        }
    }
    // Draw 4th row of keys
    yk = 4;
    CleO.Tag(10);
    CleO.eve_button(1 * (BTN_WIDTH + XGAP) - BTN_WIDTH, (yk * 60) + 20, BTN_WIDTH, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "0");
    CleO.Tag(11);
    CleO.eve_button(2 * (BTN_WIDTH + XGAP) - BTN_WIDTH, (yk * 60) + 20, BTN_WIDTH, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, ".");
    CleO.Tag(12);
    CleO.eve_button(3 * (BTN_WIDTH + XGAP) - BTN_WIDTH, (yk * 60) + 20, BTN_WIDTH, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "C");
    CleO.Tag(13);
    CleO.eve_button(5 * (BTN_WIDTH + XGAP) - BTN_WIDTH, (yk * 60) + 20, 110, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "Enter");
    // Draw "Next" button
    CleO.eve_fgcolor(LIGHT_BLUE);
    CleO.Tag(14);
    CleO.eve_button(370, 15, 110, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, ">");
    // Draw output ON/OFF button
    if (outputEnable[loopNum]) {
	    CleO.eve_fgcolor(LIGHT_GREEN);
        CleO.Tag(15);
        CleO.eve_button(370, 170, 110, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "ON");
    }
    else {
	    CleO.eve_fgcolor(ORANGE);
        CleO.Tag(15);
        CleO.eve_button(370, 170, 110, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "OFF");
    }
	CleO.StringExt(FONT_SANS_4, 335, 130, WHITE, TL, 0, 0, "Output is:");
    isKeypadDisplay = true;
    CleO.Tag(99);
}

/*
Function Displays autotune dialog
*/
void displayAtDialog() {
    isOverviewDisplay = false;
    isKeypadDisplay = false;

    String atText = "";

   // Clear Header Window
    CleO.RectangleCorner(1);
    CleO.RectangleColor(BLACK);
    CleO.RectangleXY(80, 0, 400, 80);

   // Clear Dialog Window, leave alarm status
    CleO.RectangleCorner(1);
    CleO.RectangleColor(BLACK);
    //CleO.RectangleXY(0, 78, 479, 240);
    CleO.RectangleXY(0, 78, 479, 202);

    Title1 = String(loopLabel[AT_loopNum]) + " - AutoTune";

    // Draw Title Headers
    if (!tune[AT_loopNum] && cycle == 0) {
        Title2 = "Wait for stable PV";
    }
    else {
        if (cycle < 4) {Title2 = "Tuning - Step " + String(cycle) + "/4";}
		else {Title2 = "Tuning - Finished";}
    }
    Title1.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_4, 98, 0, LIGHT_BLUE, TL, 0, 0, buf);
    Title2.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_4, 98, 35, LIGHT_BLUE, TL, 0, 0, buf);


    //atText = loopLabel[AT_loopNum] + "  -  Autotune: Cycle " + String(cycle) + "/4";
    //atText.toCharArray(buf, 60);
    //CleO.StringExt(FONT_SANS_2, 20, 90, LIGHTGREY, TL, 0, 0, buf);

    atText = "SP = " + String(int(round(Setpoint[AT_loopNum]))) + "  PV = " + String(temp[AT_loopNum]) + "  Out% = " + String(outPercent[AT_loopNum]);
    atText.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_3, 20, 85, LIGHTGREY, TL, 0, 0, buf);

    atText = "Old: Kp = " + String(Kp[AT_loopNum]) + "   Ki = " + String(Ki[AT_loopNum]);
    atText.toCharArray(buf, 60);
    CleO.StringExt(FONT_SANS_3, 20, 110, LIGHTGREY, TL, 0, 0, buf);
    if (cycle == 4) {
        atText = "New: Kp = " + String(AT_Kp) + "   Ki = " + String(AT_Ki);
        atText.toCharArray(buf, 60);
        CleO.StringExt(FONT_SANS_3, 20, 135, LIGHT_BLUE, TL, 0, 0, buf);
    }

    // Button Color
    CleO.StringExt(FONT_SANS_2, 0, 150, BLACK, TL, 0, 0, "");
    CleO.eve_fgcolor(YELLOW);
    // Start Button
    if (!tune[AT_loopNum]) {
        CleO.Tag(50);
        CleO.eve_button(20, 165, BTN_WIDTH + 25, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "Start");
    }
    // Cancel Button
    CleO.Tag(51);
    CleO.eve_button(20, 230, BTN_WIDTH + 25, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "Cancel");
    // Save Button
    if (cycle == 4) {
        CleO.Tag(52);
        CleO.eve_button(130, 230, BTN_WIDTH + 25, BTN_HEIGHT, FONT_MEDIUM, OPT_FLAT, "Save");
    }
    isAtDisplay = true;
    CleO.Tag(99);
}


/*
Function Handles Screen Touch Events
*/
void control()
{
 int16_t x, y, dur, tag;
 CleO.TouchCoordinates(x, y, dur, tag);
 if (tag != oldtag) {
    if (isOverviewDisplay) {
        // Setpoint value is pressed, show keypad to enter Loop Setpoint
        if (tag == 22 || tag == 26 || tag == 30 || tag == 34 || tag == 38) {
           loopNum = (tag - 22) / 4;
           keypadDisplay = true;
           overviewDisplay = false;
           autotuneDisplay = false;
           newEntry = true;
           targetStr[loopNum] = String(int(round(Setpoint[loopNum])));
        }
	     // Display Autotune Dialog
        if (tag == 23 || tag == 27 || tag == 31 || tag == 35 || tag == 39) {
            AT_loopNum = (tag - 23) / 4;
            autotuneDisplay = true;
            keypadDisplay = false;
            overviewDisplay = false;
            //pidLoop[AT_loopNum].SetTunings(KpTuning, 0.0, 0.0);
			pidLoop[AT_loopNum].SetMode(MANUAL);  // Disable PID
            cycle = 0;
            loopStatus[AT_loopNum] = 2;
        }
    }
    if (isAtDisplay) {
        // Start Autotune
        if (tag == 50) {
            tune[AT_loopNum] = true;
            atInit[AT_loopNum] = true;
            loopStatus[AT_loopNum] = 3;
            pidLoop[AT_loopNum].SetMode(MANUAL);
        }
        // Cancel Autotune
        if (tag == 51) {
            pidLoop[AT_loopNum].SetTunings(Kp[AT_loopNum], Ki[AT_loopNum], 0.0);
            tune[AT_loopNum] = false;
            loopStatus[AT_loopNum] = 1;
            autotuneDisplay = false;
            keypadDisplay = false;
            overviewDisplay = true;
            pidLoop[AT_loopNum].SetMode(AUTOMATIC);
        }
        // Save Autotune Parameters
        if (tag == 52) {
            tune[AT_loopNum] = false;
            Kp[AT_loopNum] = AT_Kp;
            Ki[AT_loopNum] = AT_Ki;
            pidLoop[AT_loopNum].SetTunings(AT_Kp, AT_Ki, 0.0);
            String SDpath = "AshbyCross/kp" + String(AT_loopNum) + ".txt";         // Save Kp, Ki to sd card
            save_str_to_SD(SDpath, String(AT_Kp));
            SDpath = "AshbyCross/ki" + String(AT_loopNum) + ".txt";
            save_str_to_SD(SDpath, String(AT_Ki));
            loopStatus[AT_loopNum] = 1;
            sendDataToBrowser(AT_loopNum);
            autotuneDisplay = false;
            keypadDisplay = false;
            overviewDisplay = true;
            pidLoop[AT_loopNum].SetMode(AUTOMATIC);
        }
    }
    if (isKeypadDisplay) {
        // Keys "1" through "9" pressed
        if (tag > 0 && tag < 10) {
            if (newEntry == true) {
               targetStr[loopNum] = String(tag);
               newEntry = false;
            }
            else {
               targetStr[loopNum] = targetStr[loopNum] + String(tag);
           }
        }
        // "0" Key pressed
        else if (tag == 10) {
            if (newEntry == true) {
               targetStr[loopNum] = "0";
               newEntry = false;
            }
            else {
               targetStr[loopNum] = targetStr[loopNum] + "0";
            }
        }
        // "Clear" Key pressed
        else if (tag == 12) {
            targetStr[loopNum] = "0";
            newEntry = true;
        }
        // "Enter" Button pressed
        else if (tag == 13 && keypadDisplay) {
            keypadDisplay = false;
            Setpoint[loopNum] = double(atoi(targetStr[loopNum].c_str()));                                                            // Convert string to Setpoint
            if (Setpoint[loopNum] < SpMin) {Setpoint[loopNum] = SpMin; targetStr[loopNum] = String(int(round(Setpoint[loopNum])));}  // Check low SP value
            if (Setpoint[loopNum] > SpMax) {Setpoint[loopNum] = SpMax; targetStr[loopNum] = String(int(round(Setpoint[loopNum])));}  // Check high SP value
            String SDpath = "AshbyCross/sp" + String(loopNum) + ".txt";                                                              // Save SP to sd card
            save_str_to_SD(SDpath, targetStr[loopNum]);
            delay(200);
            overviewDisplay = true;
            autotuneDisplay = false;
            keypadDisplay = false;
        }
        // "Next" Button pressed in keypad display
        else if (tag == 14) {
            loopNum = loopNum + 1;
            if (loopNum > (numLoops - 1)) {
              loopNum = 0;
            }
            newEntry = true;
            targetStr[loopNum] = String(int(round(Setpoint[loopNum])));
        }
        // Output Button pressed in keypad display
        else if (tag == 15) {
            outputEnable[loopNum] = !outputEnable[loopNum];
            if (outputEnable[loopNum] == false) {
                LoopError[loopNum] = 0;
                AlarmStatus[loopNum] = 0;
                AlarmText = "";
                LoopAlarmText = "";
				        time_equal[loopNum] = time_unequal[loopNum];
            }
        }
    }
 }
 oldtag = tag;
}

 /*
  4-20mA Output (Buzzer)
  */
 void CurrentLoop(uint16_t dac_i)
 {
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                    // cmd to update the DAC (0x40)
  Wire.write(dac_i / 16);            // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
  Wire.write((dac_i % 16) << 4);     // Lower data bits (D3.D2.D1.D0.x.x.x.x)
  Wire.endTransmission();
 }
 
/*
Function Performs PI Loop Autotune
*/
void autotune(int loopIdx) {
  static double tempMin_old[] = {500.0, 500.0, 500.0, 500.0, 500.0};
  static double tempMax_old[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  static double tempMin[] = {500.0, 500.0, 500.0, 500.0, 500.0};
  static double tempMax[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  static double baseOutput = Output[loopIdx];
  static double triggerTemp = temp[loopIdx];
  static double Output_old = Output[loopIdx];
  static unsigned long time1 = 0;
  static unsigned long time2 = 0;
  double Ku = 0.0;
  double Pu = 0.0;
  static double outUp = 0.0;
  static double outDn = 0.0;
  
  Title2 = "Tuning -- Step " + String(cycle) +"/4";
  
  if (atInit[loopIdx]) {
      for (int i = 0; i < numLoops; i++) {
         tempMin[i] = 500.0;
         tempMax[i] = 0.0;
         tempMin_old[i] = 500.0;
         tempMax_old[i] = 0.0;
      }
      baseOutput = Output[loopIdx];
      Output_old = Output[loopIdx];

      outUp = baseOutput + (baseOutput * deltaOut);
      outDn = baseOutput - (baseOutput * deltaOut);
      if (outUp > WindowSize) {
          outUp = WindowSize;
		      outDn = 0.0;
	  }

      triggerTemp = temp[loopIdx] + 1.0;

      atInit[loopIdx] = false;
      cycle = 0;
  }
  if (temp[loopIdx] < triggerTemp) {
      Output[loopIdx] = outUp;
  }
  if (temp[loopIdx] > triggerTemp) {
      Output[loopIdx] = outDn;
  }
  if (cycle > 2) {
    if (temp[loopIdx] > tempMax_old[loopIdx]) {
        tempMax[loopIdx] = temp[loopIdx];
    }
    if (temp[loopIdx] < tempMin_old[loopIdx]) {
        tempMin[loopIdx] = temp[loopIdx];
    }
    tempMax_old[loopIdx] = tempMax[loopIdx];
    tempMin_old[loopIdx] = tempMin[loopIdx];
  }

  if (Output[loopIdx] != Output_old) {
      // Output High -> LOW
      if (Output[loopIdx] < (baseOutput - (baseOutput * 0.1))) {
          cycle = cycle + 1;
          if (cycle == 3) {
            time1 = millis();
          }
          if (cycle == 4) {
            time2 = millis();
          }
      }
	  // Output LOW -> HIGH
      if (Output[loopIdx] > (baseOutput + (baseOutput * 0.1))) {
          //time2 = millis();
          //Serial.print("T2: "); Serial.println(time2);
      }
  }
  Output_old = Output[loopIdx];

  if (cycle == 4) {
    long dT = time1 - time2;
    long deltaT = abs(dT);
    Pu = float(deltaT) / 1000.0;
    Ku = 4.0 * (baseOutput * deltaOut * 2.0) / ((tempMax[loopIdx] - tempMin[loopIdx]) * 3.1415);
    AT_Kp = 0.4 * Ku;
    AT_Ki = 0.48 * Ku / Pu;
    tune[loopIdx] = false;
    Title2 = "Tuning -- Finished";
  }
}

 /*
 Write integer Data as string to sd card
 */
 void save_to_SD(String filename, int value)
 {
   int16_t file1 = CleO.FOpen(filename.c_str(), FILE_CREATE_ALWAYS | FILE_WRITE);
   itoa(value,buf,10);
   CleO.FPutS(file1, buf);
   CleO.FClose(file1);
 }
 
  /*
 Write String Data as string to sd card
 */
 void save_str_to_SD(String filename, String dataStr) {
   char sdbuf[700];
   int16_t file2 = CleO.FOpen(filename.c_str(), FILE_CREATE_ALWAYS | FILE_WRITE);
   int len = dataStr.length() + 1;
   dataStr.toCharArray(sdbuf, len);
   sdbuf[len-1] = 0;
   CleO.FPutS(file2, sdbuf);
   CleO.FClose(file2);
 }

/*
Function sets Thermocouple filter
*/
 void setFilter() {
   StatusStr1 = "Filter (" + String(coefficient) + "): ";
   for (int i = 0; i < numLoops; i++) {
     if (tempSensor[i].isConnected()) {
       delay(10);
       tempSensor[i].setFilterCoefficient(coefficient);
	   delay(10);
       if(tempSensor[i].getFilterCoefficient() == coefficient){
         StatusStr1 += String(i) + "/";
       }
     }
   }
 }

/*
Function sets Thermocouple type
*/
 void setType() {
   StatusStr2 += "Type (" + tcChar[type] + "): ";
   for (int i = 0; i < numLoops; i++) {
     if (tempSensor[i].isConnected()) {
       tempSensor[i].setThermocoupleType(type);
       if(tempSensor[i].getThermocoupleType() == type){
         StatusStr2 += String(i) + "/";
       }
       delay(20);
     }
   }
 }

/*
Function reads Startup Data from SD Card and initializes Modbus RTU
*/
void readStartupData() {
  String SDpath = "";
  for (int i = 0; i < numLoops; i++) {
      // Read PID Sp from sd card
      SDpath = "AshbyCross/sp" + String(i) + ".txt";
      file = CleO.FOpen(SDpath.c_str(), FILE_READ);
      CleO.FGetS(file, (uint8_t *)buf, 10);
      CleO.FClose(file);
      Setpoint[i] = atof(buf);
      // Read PID Kp from sd card
      SDpath = "AshbyCross/kp" + String(i) + ".txt";
      file = CleO.FOpen(SDpath.c_str(), FILE_READ);
      CleO.FGetS(file, (uint8_t *)buf, 10);
      CleO.FClose(file);
      Kp[i] = atof(buf);
      // Read PID Ki from sd card
      SDpath = "AshbyCross/ki" + String(i) + ".txt";
      file = CleO.FOpen(SDpath.c_str(), FILE_READ);
      CleO.FGetS(file, (uint8_t *)buf, 10);
      CleO.FClose(file);
      Ki[i] = atof(buf);
      // Read PID Labels from sd card
      SDpath = "AshbyCross/label" + String(i) + ".txt";
      file = CleO.FOpen(SDpath.c_str(), FILE_READ);
      CleO.FGetS(file, (uint8_t *)buf, 10);
      CleO.FClose(file);
      String LL = String(buf);
      LL.trim();
      loopLabel[i] = LL;
      // Read Low Alarms from sd card
      SDpath = "AshbyCross/al" + String(i) + ".txt";
      file = CleO.FOpen(SDpath.c_str(), FILE_READ);
      CleO.FGetS(file, (uint8_t *)buf, 10);
      CleO.FClose(file);
      lowAlarm[i] = atof(buf);
      // Read High Alarms from sd card
      SDpath = "AshbyCross/ah" + String(i) + ".txt";
      file = CleO.FOpen(SDpath.c_str(), FILE_READ);
      CleO.FGetS(file, (uint8_t *)buf, 10);
      CleO.FClose(file);
      highAlarm[i] = atof(buf);
      // Read Temperature Offset from sd card
      SDpath = "AshbyCross/toffset" + String(i) + ".txt";
      file = CleO.FOpen(SDpath.c_str(), FILE_READ);
      CleO.FGetS(file, (uint8_t *)buf, 10);
      CleO.FClose(file);
      tempOffset[i] = atof(buf);
  }
  // Read Thermocouple Type from sd card
  SDpath = "AshbyCross/tctype.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  int tcType = atoi(buf);
  type = TYPE_K;
  if (tcType == 0) type = TYPE_K;
  if (tcType == 1) type = TYPE_J;
  if (tcType == 2) type = TYPE_T;
  if (tcType == 3) type = TYPE_N;
  if (tcType == 4) type = TYPE_S;
  if (tcType == 5) type = TYPE_E;
  if (tcType == 6) type = TYPE_B;
  if (tcType == 7) type = TYPE_R;
  
  // Read filter coefficient from sd card
  SDpath = "AshbyCross/filter.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  coefficient = atoi(buf);
  // Read Kp Tuning from sd card
  SDpath = "AshbyCross/kpt.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  KpTuning = atof(buf);
  // Read Delta Output from sd card
  SDpath = "AshbyCross/deltaout.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  deltaOut = atof(buf);
  // Read number of loops from sd card
  SDpath = "AshbyCross/numloops.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  numLoops = atoi(buf);
  // Read min SP from sd card
  SDpath = "AshbyCross/spmin.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  SpMin = atof(buf);
  // Read max SP from sd card
  SDpath = "AshbyCross/spmax.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  SpMax = atof(buf);
  // Read timeout in seconds from sd card
  SDpath = "AshbyCross/tmoutsec.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  tmoutsec = atoi(buf);
  // Read SP Enable string from sd card
  SDpath = "AshbyCross/spenable.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  spEnableStr = String(buf);
  spEnableStr.trim();
  // Read AutoTune Enable string from sd card
  SDpath = "AshbyCross/atenable.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  atEnableStr = String(buf);
  atEnableStr.trim();
  // Read concecutve Loop Errors from SD card
  SDpath = "AshbyCross/sumerr.txt";
  file = CleO.FOpen(SDpath.c_str(), FILE_READ);
  CleO.FGetS(file, (uint8_t *)buf, 10);
  CleO.FClose(file);
  sumErr = atoi(buf);
}

/*
Function saves Loop Data (from browser) to sd card
*/
void saveLoopDataToSD(int ln) {
      String SDpath = "AshbyCross/label" + String(ln) + ".txt";
      save_str_to_SD(SDpath, String(loopLabel[ln]));
      SDpath = "AshbyCross/kp" + String(ln) + ".txt";         // Save Kp, Ki to sd card
      save_str_to_SD(SDpath, String(Kp[ln]));
      SDpath = "AshbyCross/ki" + String(ln) + ".txt";
      save_str_to_SD(SDpath, String(Ki[ln]));
      SDpath = "AshbyCross/al" + String(ln) + ".txt";
      save_str_to_SD(SDpath, String(lowAlarm[ln]));
      SDpath = "AshbyCross/ah" + String(ln) + ".txt";
      save_str_to_SD(SDpath, String(highAlarm[ln]));
}

/*
Function saves Tuning Data (from browser) to sd card
*/
void saveTuningDataToSD() {
      String SDpath = "AshbyCross/filter.txt";
      save_str_to_SD(SDpath, String(coefficient));
      SDpath = "AshbyCross/kpt.txt";
      save_str_to_SD(SDpath, String(KpTuning));
      SDpath = "AshbyCross/deltaout.txt";
      save_str_to_SD(SDpath, String(deltaOut));
      SDpath = "AshbyCross/spenable.txt";
      save_str_to_SD(SDpath, spEnableStr);
      SDpath = "AshbyCross/atenable.txt";
      save_str_to_SD(SDpath, atEnableStr);
}

/*
 Function Configures Access Point Mode
 */
void initWiFiAP() {
  APname = "abcdefg" + WiFi.macAddress();
  int AP_str_len = APname.length() + 1;
  char AP_char_array[AP_str_len];
  APname.toCharArray(AP_char_array, AP_str_len);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_char_array,password,3); // 192.168.4.1
  if (DEBUG == 1) Serial.println("Access Point configured !");
}


// WebServer Callback Functions
// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  if (DEBUG == 1) {Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());}
  request->send(SPIFFS, "/index.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  if (DEBUG == 1) {Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());}
  request->send(SPIFFS, "/styles.css", "text/css");
}

// Callback: send javascript file
void onJsRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  if (DEBUG == 1) {Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());}
  request->send(SPIFFS, "/script.js", "application/javascript");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  if (DEBUG == 1) {Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());}
  request->send(404, "text/plain", "Not found");
}

/*
 Function sends loop-specific data and global data to browser
*/
void sendDataToBrowser(int ln) {
    // Loop Name
    String wsTxt = "init:," + String(loopLabel[ln]) + ",";
    // Kp
    //wsTxt += String(pidLoop[ln].GetKp()) + ",";
    wsTxt += String(Kp[ln]) + ",";
    // Ki
    //wsTxt += String(pidLoop[ln].GetKi()) + ",";
    wsTxt += String(Ki[ln]) + ",";
    // Filter Coefficient
    wsTxt += String(coefficient) + ",";
    // Kp Tuning
    wsTxt += String(KpTuning) + ",";
    // Delta Output
    wsTxt += String(deltaOut) + ",";
    // SP Change enable
    wsTxt += spEnableStr + ",";
    // AutoTune enable
    wsTxt += atEnableStr + ",";
    // Send it
    wsTxt.toCharArray(buf, 64);
    ws.textAll(buf);
}

/*
 Function sends loop labels to browser
*/
void sendLabelsToBrowser() {
    // Loop Names
    String wsTxt = "labels:," + String(loopLabel[0]) + ",";
    wsTxt += String(loopLabel[1]) + ",";
    wsTxt += String(loopLabel[2]) + ",";
    wsTxt += String(loopLabel[3]) + ",";
    wsTxt += String(loopLabel[4]) + ",";
	wsTxt += String(numLoops) + ",";
    // Send it
    wsTxt.toCharArray(buf, 60);
    ws.textAll(buf);
}

/*
 Function sends loop alarm limits to browser
*/
void sendAlarmsToBrowser(int ln) {
    // Loop Names
    String wsTxt = "alarms:," + String(lowAlarm[ln]) + ",";
    wsTxt += String(highAlarm[ln]) + ",";
    // Send it
    wsTxt.toCharArray(buf, 60);
    ws.textAll(buf);
}

/*
 WebSocket Callback Function
*/
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){

  if(type == WS_EVT_CONNECT) {
    if (DEBUG == 1) Serial.println("Websocket client connection received");
    ws_connected = true;
    loopNumber = 0;
    AT_loopNum = 0;
    sendDataToBrowser(loopNumber);
	sendLabelsToBrowser();
	sendAlarmsToBrowser(loopNumber);
  } else if(type == WS_EVT_DISCONNECT) {
    if (DEBUG == 1) Serial.println("Client disconnected");
    ws_connected = false;
  }
  else if(type == WS_EVT_DATA) {
    if (DEBUG == 1) Serial.println("Data received: ");

    int res = 0;
    data[len]=0;
    String dataStr = (char*)data;

    if (DEBUG == 1) Serial.println(dataStr);
	
    if (dataStr.substring(0,7) == "filter:") {
      String fStr = dataStr.substring(7);
      coefficient = fStr.toInt();
      //setFilter();
    }
    else if (dataStr.substring(0,4) == "kpt:") {
      String kptStr = dataStr.substring(4);
      KpTuning = kptStr.toFloat();
    }
    else if (dataStr.substring(0,5) == "out%:") {
      String outStr = dataStr.substring(5);
      deltaOut = outStr.toFloat();
      //save_tuning_data = true;
    }
    else if (dataStr.substring(0,9) == "spchange:") {
      spEnableStr = dataStr.substring(9);
    }
    else if (dataStr.substring(0,6) == "atune:") {
      atEnableStr = dataStr.substring(6);
      save_tuning_data = true;
    }
    else if (dataStr.substring(0,3) == "ln:") {
      String lnStr = dataStr.substring(3);
      loopNumber = lnStr.toInt();
      sendDataToBrowser(loopNumber);
      sendAlarmsToBrowser(loopNumber);
    }
    else if (dataStr.substring(0,5) == "name:") {
      String nameStr = dataStr.substring(5);
      loopLabel[loopNumber] = nameStr;
    }
    else if (dataStr.substring(0,3) == "kp:") {
      String kpStr = dataStr.substring(3);
      Kp[loopNumber] = kpStr.toFloat();
    }
    else if (dataStr.substring(0,3) == "ki:") {
      String kiStr = dataStr.substring(3);
      Ki[loopNumber] = kiStr.toFloat();
      pidLoop[loopNumber].SetTunings(Kp[loopNumber], Ki[loopNumber], 0.0);
    }
    else if (dataStr.substring(0,6) == "lowal:") {
      String lowAlarmStr = dataStr.substring(6);
      lowAlarm[loopNumber] = lowAlarmStr.toFloat();
    }
    else if (dataStr.substring(0,7) == "highal:") {
      String highAlarmStr = dataStr.substring(7);
      highAlarm[loopNumber] = highAlarmStr.toFloat();
      save_loop_data = true;
    }
  }
}
 
