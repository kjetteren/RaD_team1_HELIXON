/*
  WiFi AP UDP sensor read

  A simple sensor read and data send through UDP acess point. This sketch 
  will create a new access point (with specified password). It will then 
  launch a new server and start sending data of the sensors. A Python 
  script needs to be ran at the same time to recieve the UDP packets.

  created 19 Nov 2024
  by Ruslan Galiullin
 */

#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include "arduino_secrets.h" 

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)
WiFiUDP udp;                      // UDP object
int status = WL_IDLE_STATUS;
unsigned int localPort = 12345;

// sensor setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 has an I2C address of 0x28 by default
Adafruit_BMP3XX bmp;
float a_x,a_y,a_z,m_x,m_y,m_z,omega_x,omega_y,omega_z,g_x,g_y,g_z,pressure,temperature;
imu::Vector<3> magneto;
imu::Vector<3> gravity;
imu::Vector<3> gyro;
imu::Vector<3> acceleration;
imu::Quaternion quat;
uint8_t s = 0;
uint8_t g = 0;
uint8_t a = 0; 
uint8_t m = 0;

static unsigned long previousMillis = 0; // Tracks the last loop execution time
const unsigned long interval = 40;       // 25 Hz = 40 ms
unsigned long currentMillis = 0;

void setup() {
  if (!bno.begin()) {
    while (1);
  }

  if (!bmp.begin_I2C()) {
    while (1); 
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  bno.setMode(OPERATION_MODE_NDOF);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    while (true);
  }

  while(true) {
    bno.getCalibration(&s, &g, &a, &m);
    if(s == 3){break;}
  }

  // Create access point:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    while (true);
  }

  // start listening to the specified port
  udp.begin(localPort);

  while (udp.parsePacket() < 1);
}

void loop() {
  currentMillis = millis();

  // Check if the interval has elapsed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    magneto = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    pressure = bmp.readPressure();
    temperature = bmp.readTemperature();
    quat = bno.getQuat();

    float data[] = {
      acceleration.x(), acceleration.y(), acceleration.z(),
      magneto.x(), magneto.y(), magneto.z(),
      gyro.x(), gyro.y(), gyro.z(),
      gravity.x(), gravity.y(), gravity.z(),
      quat.w(),quat.x(),quat.y(),quat.z(),
      pressure, temperature
    };

    udp.beginPacket("192.168.4.2", localPort);
    udp.write((uint8_t*)data, sizeof(data)); //Send float array as bytes
    udp.endPacket();
  }
}
