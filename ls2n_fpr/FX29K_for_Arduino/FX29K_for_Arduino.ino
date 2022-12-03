#include "FX29K.h"
#include "arduino_secrets.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

WiFiUDP Udp;

int status = WL_IDLE_STATUS;
#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)
unsigned int localPort = 2390;      // local port to listen on
IPAddress ip(192, 168, 0, 20);

FX29K scale(FX29K0, 0010);

void setup(){
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  delay(5000);
  Serial.println("Welcome!!");
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
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.config(ip);
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);

  Serial.println("Starting sensor");
  Wire.begin();
  scale.setWire(&Wire);
  scale.tare(1000);
  Serial.println("Let's go!");
}

void loop(){
  // See if we received a request packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet from");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    // send a reply, to the IP address and port that sent us the packet we received
    Serial.print("Sending reply\n");
    float g = scale.getGrams();
    Serial.println(g, 1);
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    String msg = String(g);
    Udp.print(msg);
    Udp.endPacket();
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
