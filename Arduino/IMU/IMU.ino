#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 has an I2C address of 0x28 by default
Adafruit_BMP3XX bmp;
float a_x,a_y,a_z,m_x,m_y,m_z,omega_x,omega_y,omega_z,g_x,g_y,g_z,q_w,q_x,q_y,q_z,pressure,temperature;
imu::Vector<3> magneto;
imu::Vector<3> gravity;
imu::Vector<3> gyro;
imu::Vector<3> acceleration;

void setup() {
  Serial.begin(115200);
  
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055 sensor. Check wiring.");
    while (1);
  }
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
    while (1); 
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  bno.setMode(OPERATION_MODE_NDOF);
  delay(1000);  
}

void loop() {
  acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  a_x = acceleration.x();
  a_y = acceleration.y();
  a_z = acceleration.z();
  magneto = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  m_x = magneto.x();
  m_y = magneto.y();
  m_z = magneto.z();
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  omega_x = gyro.x();
  omega_y = gyro.y();
  omega_z = gyro.z();
  gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  g_x = gravity.x();
  g_y = gravity.y();
  g_z = gravity.z();
  imu::Quaternion quat = bno.getQuat();
  q_w = quat.w();
  q_x = quat.x();
  q_y = quat.y();
  q_z = quat.z();
  pressure = bmp.readPressure();
  temperature = bmp.readTemperature();

  Serial.print(a_x); Serial.print(",");
  Serial.print(a_y); Serial.print(",");
  Serial.print(a_z); Serial.print(",");
  Serial.print(m_x); Serial.print(",");
  Serial.print(m_y); Serial.print(",");
  Serial.print(m_z); Serial.print(",");
  Serial.print(omega_x); Serial.print(",");
  Serial.print(omega_y); Serial.print(",");
  Serial.print(omega_z); Serial.print(",");
  Serial.print(g_x); Serial.print(",");
  Serial.print(g_y); Serial.print(",");
  Serial.print(g_z); Serial.print(",");
  Serial.print(q_w); Serial.print(",");
  Serial.print(q_x); Serial.print(",");
  Serial.print(q_y); Serial.print(",");
  Serial.print(q_z); Serial.print(",");
  Serial.print(pressure); Serial.print(",");
  Serial.println(temperature);
}

