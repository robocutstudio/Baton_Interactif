#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include <ESPmDNS.h> //This gives a reference error but doesn't prevent build
#include <WiFiUdp.h>
#include <ArduinoOTA.h> //This gives a reference error but doesn't prevent build
#include <PubSubClient.h> 
#include <Wire.h>
#include <stdlib.h>
#include <ArduinoJson.h>
#include <ArtnetWifi.h>

//IMU Libs
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Eeprom Lib
#include <EEPROM.h>

//mapping of data from imu
#include "map.h"

//set eeprom memory size (max numbs of addr)
#define EEPROM_SIZE 64 //bigger than necessary

//LEDS lib
#include <Adafruit_NeoPixel.h>


// Replace the next variables with your SSID/Password combination
const char* ssid = "TRANSVERSAL"; //"roboInterweb";//
const char* password = "VictorChristineS70";//"Amirobot2021";//

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "10.10.1.1";//"192.168.1.144";

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char * udpAddress = "10.10.1.126"; //"192.168.2.122"; //ip of the receiver
const int udpPort = 3333; //set port of choice

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;
OSCBundle bndl; //declare oscbundle

//dummy sender for Bundles
char valueBuffer[16] = "";

WiFiClient espClient;
PubSubClient client(espClient);
//const char* hostname = "BEATS-BATON"; // Does not appear
long lastMsg = 0;
char msg[50];
int value = 0;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

TaskHandle_t TaskIMU;
TaskHandle_t TaskLED;

bool serial_debug = false;
bool calibrated = false;

#pragma region LED

#define PIN 25
#define numLeds 327 //28
const int channelsPerLed = 4;
const int numberOfChannels = numLeds * channelsPerLed; // Total number of channels you want to receive (1 led = 3 channels)
Adafruit_NeoPixel leds = Adafruit_NeoPixel(numLeds, PIN, NEO_GRBW + NEO_KHZ800);
ArtnetWifi artnet;

const int startUniverse = 0; // CHANGE FOR YOUR SETUP most software this is 1, some software send out artnet first universe as 0.
const int maxUniverses = numberOfChannels / 512 + ((numberOfChannels % 512) ? 1 : 0);
bool universesReceived[maxUniverses];
bool sendFrame = 1;
int previousDataLength = 0;



void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
  sendFrame = 1;
  // set brightness of the whole strip
  if (universe == 15)
  {
    leds.setBrightness(data[0]);
    leds.show();
  }

  // read universe and put into the right part of the display buffer
  for (int i = 0; i < length / channelsPerLed; i++)
  {
    int led = i + (universe - startUniverse) * (previousDataLength / channelsPerLed);
    if (led < numLeds) {
      if (channelsPerLed == 4)
        leds.setPixelColor(led, data[i * channelsPerLed], data[i * channelsPerLed + 1], data[i * channelsPerLed + 2], data[i * channelsPerLed + 3]);
      if (channelsPerLed == 3)
        leds.setPixelColor(led, data[i * channelsPerLed], data[i * channelsPerLed + 1], data[i * channelsPerLed + 2]);
    }
  }
  previousDataLength = length;
  
  leds.show();
}

void color_set(u_int8_t r, u_int8_t g, u_int8_t b, u_int8_t w){
  for (int i = 0; i < numLeds; i++)
    {
      leds.setPixelColor(i, r,g,b,w);
    }
  leds.show();
}

void set_calibFinished(){
  color_set(0,255,0,0);
  delay(1000);
  color_set(0,0,0,0);
  delay(1000);
}

void TaskLEDcode(void * pvParameters){
  for(;;){
    artnet.read();
  }
}

#pragma endregion

#pragma region COMS

void sendMsg(String topic, String message){
  //Convert strings
  topic = "BEATS/baton/" + topic;
  char t[50];
  topic.toCharArray(t, 50);
  char m [50];
  message.toCharArray(m, 50);
  client.publish(t, m);
}

//wifi event handler

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  //WiFi.setHostname(hostname); //For whatever reason this is not working
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("BEATS/baton/led");
      client.subscribe("BEATS/baton/routing");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default: break;
  }
}

void parse_msg(char* topic, byte* payload, unsigned int length){
  
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;

  String sTopic(topic);
  Serial.println(topic);

  Serial.println(message);
  if (serial_debug){
    Serial.println(message);
  }

  if(sTopic == "BEATS/baton/routing"){
    //insert logic here
    Serial.println("routing");

    //Get variable this way
    //String yl = doc["/yl/"];
  }
}

#pragma endregion

#pragma region IMU Library
// Build-in librairy of Adafruit function to show interesting data for the IMU
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);

  sendMsg("calibration", "Please Calibrate the sensor");

}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
*/
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}
#pragma endregion

#pragma region IMU Thread
// I encapsulated all IMU related code here. It was necessery to a proper code declaration order for PlatformIO
void setup_imu()
{
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
    delay(1000);
  }

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
     Look for the sensor's unique ID at the beginning oF EEPROM.
     This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
  else
  {
    Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
    calibrated = true;
  }

  delay(1000);

  /* Display some basic information on this sensor */
  if (serial_debug) displaySensorDetails(); //optinal

  /* Optional: Display current status */
  if (serial_debug) displaySensorStatus(); //optinal

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration very often */
  if (foundCalib) {
    Serial.println("Move sensor slightly to calibrate magnetometers");
    while (!bno.isFullyCalibrated())
    {
      displayCalStatus(); //show calib status for each sensors+system
      Serial.println();

      bno.getEvent(&event);
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }
  else
  {
    Serial.println("Please Calibrate Sensor: ");
    //I'm often kept in this loop and can't get out
    while (!bno.isFullyCalibrated())
    {
      bno.getEvent(&event);

      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);

      /* Optional: Display calibration status */
      displayCalStatus();

      /* msg for calibration undone */
      Serial.print("\t Calibration not finish...");

      /* New line for the next sample */
      Serial.println("");

      /* Wait the specified delay before requesting new data */
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  EEPROM.commit();
  Serial.println("Data stored to EEPROM.");

  Serial.println("\n--------------------------------\n");

  sendMsg("calibration", "Calibrated");
  delay(500);
}

void sendOSCbundle() {
  udp.beginPacket(udpAddress, udpPort);
  bndl.send(udp);
  udp.endPacket();
  bndl.empty();
}

void TaskIMUcode(void * pvParameters)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  for (;;) { //infinite loop
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /*************************************************/
    /*
        Orientation
    */
    /*************************************************/
    if(euler.x() >= 180) bndl.add("/yl/").add(getOrientationX(euler.x()));
    else bndl.add("/yr/").add(getOrientationX(euler.x()));
    
    if(euler.y() >= 0) bndl.add("/pu/").add(getOrientationY(euler.y()));
    else bndl.add("/pd/").add(getOrientationY(euler.y()));
    
    if(euler.z() >= 0) bndl.add("/rl/").add(getOrientationZ(euler.z()));
    else bndl.add("/rr/").add(getOrientationZ(euler.z()));
    /*************************************************/
    /*
        ACCELEROMETER
    */
    /*************************************************/
    // value 2 is arbitrary to remove noise and bad data, might need more testing with the physical staff
    if(accel.x() >= 2) bndl.add("/ab/").add(getAccelX(accel.x()));
    else if(accel.x() <= -2) bndl.add("/af/").add(getAccelX(accel.x()));
    else {
      bndl.add("/ab/").add(0.00);
      bndl.add("/af/").add(0.00);
    }

    if(accel.y() >= 2) bndl.add("/al/").add(getAccelY(accel.y()));
    else if(accel.y() <= -2) bndl.add("/ar/").add(getAccelY(accel.y()));
    else{
      bndl.add("/al/").add(0.00);
      bndl.add("/ar/").add(0.00);
    }

    if(accel.z() >= 2) bndl.add("/au/").add(getAccelZ(accel.z()));
    else if(accel.z() <= -2) bndl.add("/ad/").add(getAccelZ(accel.z()));
    else {
      bndl.add("/au/").add(0.00);
      bndl.add("/ad/").add(0.00);
    }
    /*************************************************/
    /*
        Gyroscope
    */
    /*************************************************/
    if(gyro.x() >= 5) bndl.add("/gl/").add(getGyroX(gyro.x()));
    else if(gyro.x() <= -5) bndl.add("/gr/").add(getGyroX(gyro.x()));
    else {
      bndl.add("/gl/").add(0.00);
      bndl.add("/gr/").add(0.00);
    }

    if(gyro.y() >= 5) bndl.add("/gu/").add(getGyroY(gyro.y()));
    else if(gyro.y() <= -5) bndl.add("/gd/").add(getGyroY(gyro.y()));
    else {
      bndl.add("/gu/").add(0.00);
      bndl.add("/gd/").add(0.00);
    }

    if(gyro.z() >= 5) bndl.add("/gf/").add(getGyroZ(gyro.z()));
    else if(gyro.z() <= -5) bndl.add("/gb/").add(getGyroZ(gyro.z()));
    else {
      bndl.add("/gb/").add(0.00);
      bndl.add("/gf/").add(0.00);
    }

    /*Send OSC Bundle*/
    sendOSCbundle();
    
    /* Wait the specified delay before requesting new data */
    vTaskDelay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

#pragma endregion

#pragma region TASKS
// This setup both task on specific their specific cores
void setup_taskLed(){ 
  xTaskCreatePinnedToCore(
                    TaskLEDcode,   /* Task function. */
                    "TaskLED",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TaskLED,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 
}

void setup_taskIMU(){//Only one task (on Core 1) for the IMU is needed since Main Loop runs on Core 0
  //Create a task pined to a specific core
  xTaskCreatePinnedToCore(
                    TaskIMUcode,   /* Task function. */
                    "TaskIMU",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TaskIMU,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
}

#pragma endregion

void setup() {
  Serial.begin(115200);
  leds.begin();
  color_set(0,0,0,0); //set pixels to off
  setup_wifi();
  artnet.begin();
  artnet.setArtDmxCallback(onDmxFrame);
  client.setServer(mqtt_server, 1883);
  client.setCallback(parse_msg);
  reconnect();
  color_set(255,0,0,0); //Calibration color (red)
  setup_imu();
  setup_taskIMU();
  set_calibFinished();
  setup_taskLed(); 
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();
}