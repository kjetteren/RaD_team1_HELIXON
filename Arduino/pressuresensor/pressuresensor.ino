#include <Adafruit_BMP3XX.h>
#include <math.h>

Adafruit_BMP3XX bmp;

bool firstmeasurement = true;
float M = 0.0289644; // Molar mass of Earth's air in kg/mol 
float g = 9.80665; // Gravitational acceleration in m/s^2
float R = 8.3144598; // Universal gas constant in J/(mol·K)
float P_0, T, P, h; 
int count = 0;

void setup() {
  Serial.begin(115200);

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
    while (1); 
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
}


void loop() {

  if(firstmeasurement) {
      P_0 = bmp.readPressure(); 
      delay(500);
      P_0 = bmp.readPressure(); 
      firstmeasurement = false; 
    }
    
  T = bmp.readTemperature(); 
  P = bmp.readPressure();     
  
  h = -(R * (T + 273.15)) / (M * g) * log(P / P_0);
  Serial.print("Initial pressure: ");
  Serial.println(P_0);
  Serial.print("Temperature: ");
  Serial.println(T);
  Serial.print("Pressure: ");
  Serial.println(P);
  Serial.print("Height using barometric formula: ");
  Serial.println(h);
  Serial.print("Height using Adafruit library: ");
  Serial.println((bmp.readAltitude(P_0/100))); 
  delay(100);
}

