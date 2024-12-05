
#define BLYNK_TEMPLATE_ID "TMPL6BlkfR5q2"
#define BLYNK_TEMPLATE_NAME "Test"
#include <Arduino.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <TFT_eSPI.h> 
#include <WiFiManager.h>
#include <BlynkSimpleEsp32.h>
#include "DFRobot_BloodOxygen_S.h"
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <string.h>
#include <HCSR04.h>
#include <EEPROM.h>
BlynkTimer timer;
const char DEVICE_LOGIN_NAME[]  = "1ab0913f-8be3-4099-8618-a8668b1ec540";
const char DEVICE_KEY[]  = "hEIx0H0sgNQn31FZ4Wq5ciC5P";    // Secret device password
const char SSID[]               = "P501";    // Network SSID (name)
const char PASS[]               = "22446688";    // Network password (use for WPA, or use as key for WEP)
#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);
#endif
#define DISTANCE_ENABLE_BUTTON 34
#define WIFI_CONFIGURATION_BUTTON 39
#define LEFT3_HCSR04_TRIG 33
#define LEFT3_HCSR04_ECHO 25
#define RIGHT2_HCSR04_TRIG 26
#define RIGHT2_HCSR04_ECHO 27
#define IN_FRONT1_HCSR04_TRIG 18
#define IN_FRONT1_HCSR04_ECHO 19
#define BEHIND_HCSR04_TRIG 23
#define BEHIND_HCSR04_ECHO 32
#define EEPROM_SIZE 1
#define HCSR04_TIME_OUT 5000

static char ssid[32];
static char password[64];
const int buzzer = 5;
static SCD4x co2Sensor;
static TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
static UltraSonicDistanceSensor inFrontDistance(IN_FRONT1_HCSR04_TRIG, IN_FRONT1_HCSR04_ECHO);  //1
static UltraSonicDistanceSensor rightDistance(RIGHT2_HCSR04_TRIG, RIGHT2_HCSR04_ECHO);          //2
static UltraSonicDistanceSensor leftDistance(LEFT3_HCSR04_TRIG, LEFT3_HCSR04_ECHO);               //3
static UltraSonicDistanceSensor behindDistance(BEHIND_HCSR04_TRIG, BEHIND_HCSR04_ECHO);               //3
static WiFiManager wm;
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
static TinyGPSPlus gps;

static SoftwareSerial ss(RXPin, TXPin);
long long prevTime = 0;
static volatile bool reConfigWifi = false;
static float gPSLat;
static float gPSLong;
static float temperature;
static int cO2;
static int heartrate;
static int humidity;
static int spO2;
static bool resetDevice = false;
static bool distanceEnable = false;
static CloudLocation location;
static WiFiConnectionHandler ArduinoIoTPreferredConnection(ssid, password);
static bool isWiFiConnected = false;
static bool mydeviceReset = false;

static void wifiManagerInit(void);
static void reConfigureWifiISR(void);
static void sensorUpdateInterval();
static void tftScreenSetup(void);
static void tftUpdateValue(void);
static void preferredConnectionHandler(char *in_ssid, char *in_pass);
void onDistanceEnableChange();
void onResetDeviceChange();

static void preferredConnectionHandler(char *in_ssid, char *in_pass) {
  strcpy(ssid, in_ssid);
  strcpy(password, in_pass);
}

void onResetDeviceChange()  {
  mydeviceReset = true;
}

void initProperties(){
  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(temperature, READ,ON_CHANGE, NULL);
  ArduinoCloud.addProperty(cO2, READ, READ,ON_CHANGE, NULL);
  ArduinoCloud.addProperty(heartrate, READ,ON_CHANGE, NULL);
  ArduinoCloud.addProperty(humidity, READ,ON_CHANGE, NULL);
  ArduinoCloud.addProperty(spO2, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(resetDevice, READWRITE, ON_CHANGE, onResetDeviceChange);
  ArduinoCloud.addProperty(location, READ, 10 * SECONDS, NULL);
}

static void wifiManagerInit(void) {
  bool res = true;
  static String ssid_str, pass_str;
  isWiFiConnected = false;
  EEPROM.write(0, isWiFiConnected);
  EEPROM.commit();
  res = wm.autoConnect("SmartWheelchair","12345678"); // password protected ap
  if(WiFi.status() == WL_CONNECTED) {
    ssid_str = wm.getWiFiSSID(false);
    pass_str = wm.getWiFiPass(false);
    int ssid_len = ssid_str.length() + 1;
    ssid_str.toCharArray(ssid, ssid_len);
    int pass_len = pass_str.length() + 1;
    pass_str.toCharArray(password, pass_len);
    isWiFiConnected = true;
    EEPROM.write(0, isWiFiConnected);
    EEPROM.commit();
    Serial.println("WIFI Connected");
    preferredConnectionHandler(ssid, password);
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  }
  if (!res)
  {
    delay (5000);
  }
}

static void reConfigureWifi(void) {
  reConfigWifi = true;
}

static void co2SensorMeasurement(void) {
  if(co2Sensor.readMeasurement()) {
    cO2 = co2Sensor.getCO2();
    temperature = co2Sensor.getTemperature();
    humidity = co2Sensor.getHumidity();
    // Serial.println("C02: " + String(cO2) + " ppm");
    // Serial.println("Temperature: " + String(temperature) + " *C");
    // Serial.println("Humidity: " + String(humidity) + " %");
  }
}
static void heartRateMeasurement(void) {
  bool ret;
  ret = MAX30102.getHeartbeatSPO2();
  if(!ret) {
    return;
  } else {
    spO2 = MAX30102._sHeartbeatSPO2.SPO2;
    // Serial.println("SPO2 : " + String(spO2) + " %");
    heartrate = MAX30102._sHeartbeatSPO2.Heartbeat;
    // Serial.println("Heart Rate : " + String(heartrate) + " bbp");
  }
}

static void getLocation(void) {
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        if (gps.location.isValid())
        {
          gPSLat = gps.location.lat();
          gPSLong = gps.location.lng();            
          if((millis() - prevTime) > 10000) {
            location = {gPSLat, gPSLong};
            Serial.println("Location: Lat " + String(gPSLat) + " Long: " + String(gPSLong));
            prevTime = millis();
          }
        }
      }
    }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

static void distanceHandler(void) {
  double dInFront, dLeft, dRight, dBehind;

  dInFront = inFrontDistance.measureDistanceCm();
  dLeft = leftDistance.measureDistanceCm();
  // Serial.println("Distance: " + String(dLeft));
  dRight = rightDistance.measureDistanceCm();
  dBehind = behindDistance.measureDistanceCm();

  if((dBehind < double(40) && dBehind > double(5)) || (dInFront < double(20) && dInFront > double(5)) || (dLeft < double(20) && dLeft > double(5))  || (dRight < double(20) && dRight > double(5))) {
    // Serial.println("Distance: INFRONT: " + String(dInFront) + " LEFT: " + String(dLeft) + " RIGHT: " + String(dRight) + " BEHIND: " + String(dBehind));
    int count = 0;
    while(count < 5) {
      Serial.println("WARNING....");
      tone(buzzer, 1000); // Send 1KHz sound signal...
      delay(100);         // ...for 1 sec
      noTone(buzzer);     // Stop sound...
      count++;
    }
  }
  delay(100);
}

static void sensorUpdateInterval() {
  co2SensorMeasurement();
  heartRateMeasurement();
  tftScreenUpdateValue();
}

static void tftScreenSetup(void)
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, 320, 240, TFT_YELLOW);
  tft.fillRect(170, 27, 2, 178, TFT_YELLOW);
  tft.setCursor(80, 7, 1);
  tft.setTextSize(2);
  tft.print("Smart Wheelchair");
  tft.fillRect(0, 27, 320, 2, TFT_YELLOW);
  tft.setCursor(20, 52, 1);
  tft.print("Heart Rate");
  tft.setCursor(20, 82, 1);
  tft.print("SpO2");
  tft.setCursor(20, 112, 1);
  tft.print("Temperature");
  tft.setCursor(20, 142, 1);
  tft.print("Humidity");
  tft.setCursor(20, 172, 1);
  tft.print("CO2");
  tft.setCursor(15, 218, 1);
  tft.setTextSize(2);
  tft.print("WiFi:");
  tft.setCursor(130, 218, 1);
  tft.setTextSize(2);
  tft.print("Warning:");
  tft.fillRect(0, 205, 320, 2, TFT_YELLOW);
  tft.fillRect(120, 205, 2, 35, TFT_YELLOW);
}

static void tftScreenUpdateValue(void)
{
  tft.fillRect(215, 40, 100, 30, TFT_BLACK);
  tft.setCursor(215, 52, 1);
  tft.print(String(heartrate) + " bpp");
  tft.fillRect(215, 70, 100, 30, TFT_BLACK);
  tft.setCursor(215, 82, 1);
  tft.print(String(spO2) + " %");
  tft.fillRect(215, 100, 100, 30, TFT_BLACK);
  tft.setCursor(215, 112, 1);
  tft.print(String(temperature) + " ÷C");
  tft.fillRect(215, 130, 100, 30, TFT_BLACK);
  tft.setCursor(215, 142, 1);
  tft.print(String(humidity) + " %");
  tft.fillRect(215, 160, 100, 30, TFT_BLACK);
  tft.setCursor(215, 172, 1);
  tft.print(String(cO2) + " ppm");
  if(isWiFiConnected) {
      tft.fillCircle(90, 223, 10, TFT_GREEN);
  } else {
      tft.fillCircle(90, 223, 10, TFT_RED);
  }
  if(distanceEnable) {
    tft.fillRect(235, 212, 60, 22, TFT_GREEN);
  } else {
    tft.fillRect(235, 212, 60, 22, TFT_RED);
  }
}

void setup() {
  // put your setup code here, to run once:
  static String ssid_str, pass_str;
  Serial.begin(115200);
  ss.begin(GPSBaud);
  EEPROM.begin(EEPROM_SIZE);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  pinMode(DISTANCE_ENABLE_BUTTON, INPUT);
  Wire.begin();
  if(!co2Sensor.begin()) {
    Serial.println("CO2 Sensor init failed!");
    while(1){};
  }
    Serial.println("CO2 Sensor init done.");
  if(!MAX30102.begin()) {
    Serial.println("Heart Rate sensor init failed!");
    while(1) {};
  }
  Serial.println("Heart Rate sensor init done.");
  MAX30102.sensorStartCollect();
  isWiFiConnected = EEPROM.read(0);
  tftScreenSetup();
  initProperties();
  if(isWiFiConnected) {
    if(WiFi.status() == WL_CONNECTED) {
      ssid_str = wm.getWiFiSSID(false);
      pass_str = wm.getWiFiPass(false);
      int ssid_len = ssid_str.length() + 1;
      ssid_str.toCharArray(ssid, ssid_len);
      int pass_len = pass_str.length() + 1;
      pass_str.toCharArray(password, pass_len);
      isWiFiConnected = true;
      EEPROM.write(0, isWiFiConnected);
      EEPROM.commit();
      Serial.println("WIFI Connected");
    }
    preferredConnectionHandler(ssid, password);
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  }
  timer.setInterval(2000L, sensorUpdateInterval);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  attachInterrupt(digitalPinToInterrupt(WIFI_CONFIGURATION_BUTTON), reConfigureWifi, RISING);
}

void loop() {
  if(isWiFiConnected) {
    ArduinoCloud.update();
  }
  timer.run();
  if(reConfigWifi) {
    isWiFiConnected = false;
    EEPROM.write(0, isWiFiConnected);
    EEPROM.commit();
    wm.resetSettings();
    wifiManagerInit();
    // preferredConnectionHandler(ssid,password);
    // ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    reConfigWifi =  false;
  }// ...for 1sec
  distanceEnable = (digitalRead(DISTANCE_ENABLE_BUTTON) == HIGH) ? true : false;
  if(distanceEnable) {
    distanceHandler();
  }
  getLocation();
  if(mydeviceReset) {
    ESP.restart();
  }
}