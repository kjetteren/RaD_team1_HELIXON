/*
  WiFi AP UDP packet loss test

  A simple UDP packet loss test. This sketch will create a new access point
  (with specified password). It will then launch a new server and print out 
  the IP address to the Serial Monitor. A Python script needs to be ran at
  the same time to send the UDP packets.

  created 12 Nov 2024
  by Ruslan Galiullin
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key index number (needed only for WEP)
WiFiUDP udp;                      // UDP object
unsigned int localPort = 12345;   // Port to listen on

int status = WL_IDLE_STATUS;

int receivedPackets = 0;          // Counts received packets
int expectedPackets = 100;        // Expected number of packets to receive

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting UDP Packet Loss Test");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start listening to the specified port
  udp.begin(localPort);
  Serial.println("UDP server started, waiting for packets...");

  // you're connected now, so print out the status
  printWiFiStatus();
}


void loop() {
  int packetSize = udp.parsePacket(); // Check for incoming packets

  if (packetSize) {
    // Packet received
    receivedPackets++;
    Serial.print("Received packet #");
    Serial.println(receivedPackets);

    // Clear buffer and receive data (optional, for packet integrity check)
    char packetBuffer[255];
    udp.read(packetBuffer, 255); // Read incoming packet

    // Calculate and display packet loss percentage if we reach the expected count
    if (receivedPackets >= expectedPackets) {
      float packetLossPercent = 100.0 * (1.0 - (float(receivedPackets) / float(expectedPackets)));
      Serial.print("Packet loss: ");
      Serial.print(packetLossPercent);
      Serial.println("%");

      // Reset count for next round of tests
      receivedPackets = 0;
    }
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}
