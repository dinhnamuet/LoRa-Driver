/**
 * ESP32 Dev Kit
 * Just for testing
 */
#include <SPI.h>
#include <LoRa.h>

#define SS 15
#define RST 4
#define DIO0 2

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.println("LoRa Receiver");
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.onReceive(onReceive);
  LoRa.receive();
}

void loop() {
}

void onReceive(int packetSize) {
  Serial.print("Received packet '");
  for (int i = 0; i < packetSize; i++) {
    Serial.print((char)LoRa.read());
  }
  Serial.print("' with RSSI ");
  Serial.print(LoRa.packetRssi());
  Serial.print(", SNR ");
  Serial.println(LoRa.packetSnr());
}