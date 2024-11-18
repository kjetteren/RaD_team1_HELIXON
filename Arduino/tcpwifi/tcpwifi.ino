/*
  WiFi AP TCP packet loss test

  A simple TCP packet loss test. This sketch will create a new access point
  (with specified password). It will then launch a new server and print out
  the IP address to the Serial Monitor. A Python script needs to be ran at
  the same time to send the TCP packets.

  created 12 Nov 2024
  by Ruslan Galiullin
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)
WiFiServer server(12345);         // TCP server on port 12345
WiFiClient client;                // TCP client object

int status = WL_IDLE_STATUS;

int receivedPackets = 0;          // Counts received packets
int expectedPackets = 255;        // Expected number of packets to receive

unsigned long lastPacketTime = 0; // Timestamp of the last received packet
unsigned long timeout = 5000;     // Timeout threshold in milliseconds (5 seconds)

void setup() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    // don't continue
    while (true);
  }

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    // don't continue
    while (true);
  }

  // wait 2 seconds for connection:
  delay(2000);

  // starting the server
  server.begin();
}


void loop() {
  WiFiClient client = server.available();

  if (client) {
    // reset timeout counter when a client is connected
    lastPacketTime = millis();
    
    while (client.connected()) {
      if (client.available()) {
        char rbyte = client.read(); // read incoming byte
        if (rbyte == receivedPackets) {
          receivedPackets++;
          // reset timeout if we received a valid packet
          lastPacketTime = millis();
        }
      }

      // check for timeout
      if (millis() - lastPacketTime > timeout) {
        // timeout occurred, assume packets are lost
        client.write('1');

        // reset receivedPackets to start counting new packets
        receivedPackets = 0;

        // reset lastPacketTime to continue checking for new packets
        lastPacketTime = millis();
      }

      // calculate packet loss once the expected number of packets is received
      if (receivedPackets >= expectedPackets) {
        float packetLossPercent = 100.0 * (1.0 - (float(receivedPackets) / float(expectedPackets)));

        // send packet loss status (1 for loss, 0 for no loss)
        if (packetLossPercent > 0) {
          client.write('1');  // Send '1' if packet loss occurred
        } else {
          client.write('0');  // Send '0' if no packet loss occurred
        }

        // reset receivedPackets and lastPacketTime for the next batch of packets
        receivedPackets = 0;
        lastPacketTime = millis();
      }
    }
    client.stop(); // disconnect after client finishes sending packets
  }
}
