#include "SPI.h"
#include "LoRa.h"


#define SCK 14
#define CS 15 
#define MOSI 26
#define MISO 25
#define RST 19
#define DIO0 21

int counter = 0;


void setup(){
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  LoRa.setPins(CS, RST, DIO0);
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while(!LoRa.begin(433E6)) {
    Serial.println("Error starting LoRa - Restart after 5s");
    delay(5000);
  }

  
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Transmitter Initializing OK!");
}
 
void loop(){
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();
  counter++;
  
  delay(1000);
}
