#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>

#define SPEAKER_PIN 14        // Define the pin number for the servo motor
#define LED_PIN 2             // ESP32 built-in LED pin (GPIO 2)
#define SENSITIVITY 100       // Define the sensitivity for detecting significant acceleration change
#define SCAN_INTERVAL_MS 1000 // Scan interval for Bluetooth device check (1 second)

// ----------------
// Set your WiFi SSID and Password here
// ----------------
const char *ssid = "r3d3";
const char *password = "12345678";

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 0, 1);
DNSServer dnsServer;
WebServer webServer(80);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
float lastAcceleration = 0;
bool bluetoothDeviceFound = false;

const char *root =
#include "html/root.h"
    ;

void setup()
{
  Serial.begin(115200);

  // Initialize Wifi Ad-Hoc Network
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);

  webServer.on("/", []()
               { webServer.send(200, "text/html", root); });

  webServer.on(
      "/update", HTTP_POST, []()
      {
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); },
      []()
      {
        HTTPUpload &upload = webServer.upload();
        if (upload.status == UPLOAD_FILE_START)
        {
          Serial.printf("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          { // start with max available size
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
          /* flashing firmware to ESP*/
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
          {
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (Update.end(true))
          { // true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          }
          else
          {
            Update.printError(Serial);
          }
        }
      });

  server.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  randomSeed(analogRead(0)); // Seed the random number generator

  // Initialize the LSM303 accelerometer
  if (!accel.begin())
  {
    Serial.println("LSM303 not detected. Check wiring!");
    while (1)
      ;
  }
}

void loop()
{
  // Check for Bluetooth device nearby
  checkBluetoothDevice();

  // Get accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  // Calculate the magnitude of acceleration
  float acceleration = sqrt(event.acceleration.x * event.acceleration.x +
                            event.acceleration.y * event.acceleration.y +
                            event.acceleration.z * event.acceleration.z);

  Serial.print("Acceleration: ");
  Serial.println(acceleration);

  // Compare current acceleration with the last one
  float accelerationChange = abs(acceleration - lastAcceleration);
  lastAcceleration = acceleration;

  // Check if the acceleration change exceeds the sensitivity threshold
  if (accelerationChange > SENSITIVITY && !bluetoothDeviceFound)
  {
    // Object is being stolen! Activate the servo motor and blink the LED.
    int frequency = random(200, 4001);

    // Sound the alarm for 1 second
    tone(SPEAKER_PIN, frequency);
    delay(1000); // Sound the alarm for 1 second

    // Turn off the speaker
    noTone(SPEAKER_PIN);
  }

  webServer.handleClient();
  delay(1);
}

void checkBluetoothDevice()
{
  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults foundDevices = pBLEScan->start(SCAN_INTERVAL_MS);
  for (int i = 0; i < foundDevices.getCount(); i++)
  {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    // Replace "YourDeviceName" with the name of the specific Bluetooth device you are looking for
    if (device.haveName() && device.getName() == "YourDeviceName")
    {
      bluetoothDeviceFound = true;
      return;
    }
  }
  bluetoothDeviceFound = false;
}

void blinkLED(int times)
{
  for (int i = 0; i < times * 2; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}
