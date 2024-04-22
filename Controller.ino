#include <WiFi.h>
#include <SPI.h>
#include <WiFiUdp.h>
#include "sbus.h"

char ssid[] = "NETGEAR08"; // WiFi Network Name
char pass[] = ""; // WiFi Network Password
int status = WL_IDLE_STATUS;

unsigned int localPort = 2390; // Local port to listen for UDP messages

char packetBuffer[255]; // Buffer for incoming packets
char replyBuffer[] = "acknoledged";

WiFiUDP Udp;

bfs::SbusTx sbus(&Serial1, false); // SBus Tx object for communication over UART1 (GP4)
bfs::SbusData data; // Sbus data object

void setup() {
  //Serial.begin(115200);
  sbus.Begin();
  //while(!Serial); // Wait for serial monitor to open

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Attempt to connect to WiFi network
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }

  //Serial.print("Connected to network");
  printWifiStatus();

  Serial.println("\nStarting Connection to server...");
  Udp.begin(localPort);
  blink_on();
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Read packet into buffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    // Print packet contents
    Serial.println("Contents: ");
    Serial.println(packetBuffer);

    // Send a reply
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(replyBuffer);
    Udp.endPacket();

    // Write the information from the collected data to an sbus packet    
    char * strtokIndx;
    char tempBuffer[255];
    strcpy(tempBuffer, packetBuffer); // Copy the received packet
    
    // Parse the packet, writing it into the SBus data
    strtokIndx = strtok(tempBuffer, "\t");
    for (int i = 0; i <=3; i++) {
      int val = atoi(strtokIndx);
      data.ch[i] = val;
      strtokIndx = strtok(NULL, "\t");
    }

    sbus.data(data);
    sbus.Write();
  }
}

void printWifiStatus() {
  // Print Network Name
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print IP Address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print Signal Strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void blink_on() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(600);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(600);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(600);
  digitalWrite(LED_BUILTIN, LOW);
  delay(600);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(600);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
}
