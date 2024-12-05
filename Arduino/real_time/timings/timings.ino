#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include "arduino_secrets.h"

char secret_ssid[] = SECRET_SSID;
char secret_pass[] = SECRET_PASS;
int keyIndex = 0;
WiFiUDP udp;

int status = WL_IDLE_STATUS;
unsigned int localPort = 12345;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;

float pressure, temperature;
imu::Quaternion quat;

unsigned long previousMillis = 0;
const unsigned long interval = 40; // 25 Hz = 40 ms
unsigned long currentMillis = 0;

float pc_start_time = 0.0; // Python's start time received during setup

void setup() {
  if (!bno.begin()) {
    while (1); // Halt if BNO055 fails to initialize
  }

  if (!bmp.begin_I2C()) {
    while (1); // Halt if BMP3XX fails to initialize
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  bno.setMode(OPERATION_MODE_NDOF);

  if (WiFi.status() == WL_NO_MODULE) {
    while (true);
  }

  status = WiFi.beginAP(secret_ssid, secret_pass);
  if (status != WL_AP_LISTENING) {
    while (true);
  }

  udp.begin(localPort);

  while (udp.parsePacket() < 1); // Wait for Python to send start time
  udp.read((uint8_t*)&pc_start_time, sizeof(pc_start_time)); // Receive Python's start time
}

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  
    pressure = bmp.readPressure();
    temperature = bmp.readTemperature();
    quat = bno.getQuat();

    float adjusted_time = pc_start_time + (currentMillis / 1000.0);

    float data[] = {
      quat.w(), quat.x(), quat.y(), quat.z(),
      pressure, temperature, adjusted_time
    };

    udp.beginPacket("192.168.4.2", localPort);
    udp.write((uint8_t*)data, sizeof(data));
    udp.endPacket();
  }
}
