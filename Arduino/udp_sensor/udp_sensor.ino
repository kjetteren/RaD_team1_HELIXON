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
unsigned int localPort = 12345;   // Port to listen on
int status = WL_IDLE_STATUS;
IPAddress udpAddress;

// sensor setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 has an I2C address of 0x28 by default
Adafruit_BMP3XX bmp;
float a_x,a_y,a_z,m_x,m_y,m_z,omega_x,omega_y,omega_z,g_x,g_y,g_z,pressure,temperature;
imu::Vector<3> magneto;
imu::Vector<3> gravity;
imu::Vector<3> gyro;
imu::Vector<3> acceleration;

void setup() {
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  bno.setMode(OPERATION_MODE_NDOF);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    // don't continue
    while (true);
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    // don't continue
    while (true);
  }

  // wait 5 seconds for connection:
  delay(5000);

  // start listening to the specified port
  udp.begin(localPort);
}

void loop() {
  WiFiClient client = WiFi.available();

  if (client) {
    udpAddress = client.remoteIP(); // get client's IP

    acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    magneto = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    pressure = bmp.readPressure();
    temperature = bmp.readTemperature();

    float data[] = {
      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
      gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
      gravity.acceleration.x, gravity.acceleration.y, gravity.acceleration.z,
      pressure, temperature
    };

    // Send data via UDP
    udp.beginPacket(udpAddress, localPort);
    udp.write((uint8_t*)data, sizeof(data)); // Send float array as bytes
    udp.endPacket();
  }

  delay(40);
}
