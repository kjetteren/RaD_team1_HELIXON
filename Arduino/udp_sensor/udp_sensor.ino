/*
  WiFi AP UDP sensor read

  A simple sensor read and data send through UDP acess point. This sketch 
  will create a new access point (with specified password). It will then 
  launch a new server and start sending data of the sensors. A Python 
  script needs to be ran at the same time to recieve the UDP packets.

  created 19 Nov 2024
  by Ruslan Galiullin
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)
WiFiUDP udp;                      // UDP object
unsigned int localPort = 12345;   // Port to listen on

int status = WL_IDLE_STATUS;

void setup() {
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
  // put your main code here, to run repeatedly:

}
