#include <Arduino.h>
#include <LinkedList.h>
#include <LoRa.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include "BT_Packet.h"
#include "BT_UI.h"
#include "Buddy.h"


#define DEBUG_MODE      true
#define MAX_UINT8       255
#define LARGEST_INDEX   MAX_UINT8 - 1
#define LoRa_csPin      8
#define LoRa_resetPin   4
#define LoRa_irqPin     3
#define UI_pin          6


#ifndef UNIT_TEST


// TODO: equator case
// TODO: when not moving, transmit less
// TODO: small angle approximation based on max RSSI


void onReceive(int packetSize);
void sendPacket(BT_Packet packet);
void updateBuddy(uint64_t UUID, uint16_t lat, uint16_t lng);
void updateUI();
uint8_t findBuddyBy(uint64_t UUID);


// TODO: define comparator for sorting LinkedList
LinkedList<Buddy*> buddies;

uint32_t timer = millis();
const int32_t LAT_LNG_ERR = 999999999;
uint32_t buddyColor;
const uint16_t fieldOfView = 360;
uint8_t numLEDs = 7; // must / should be an odd number
// minus 1 because of center LED, plus 0.5 so that edge LED has same "width" as others
uint8_t degreesPerLED = fieldOfView / (numLEDs - 1 + 0.5);


// TEST VALUES
uint64_t myUUID = 12345;
int32_t myLat = 53631611;
int32_t myLng = -113323975;
// TEST VALUES
//uint64_t myUUID = 106;
//int32_t myLat = 53631612;
//int32_t myLng = -113323975;
// TEST VALUES
//uint64_t myUUID = ?;
//int32_t myLat = LAT_LNG_ERR;
//int32_t myLng = LAT_LNG_ERR;
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

TinyGPSPlus gps;
BT_Packet myPacket(myUUID, myLat, myLng);
BT_UI userInterface(UI_pin);

//Uart GPSSerial (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
//Adafruit_GPS GPS(&GPSSerial);


void setup() {
    Serial.begin(9600);
    //startGPS();
    while (!Serial);

    Serial.println("BuddyTracker");

    LoRa.setPins(LoRa_csPin, LoRa_resetPin, LoRa_irqPin);
    if (!LoRa.begin(915E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    userInterface.begin();
    //userInterface.setBrightness(100);
    if(DEBUG_MODE) userInterface.test();

    buddyColor = userInterface.Color(LED_OFF, LED_OFF, LED_ON);
}


void loop() {
    // TODO: collision avoidance

    if (millis() - lastSendTime > interval) {
        sendPacket(myPacket);
        Serial.println("Sending...");
        lastSendTime = millis();            // timestamp the message
        interval = random(2000) + 1000;    // 2-3 seconds
    }
    
    // parse for a packet, and call onReceive with the result:
    onReceive(LoRa.parsePacket());
    
    updateUI();
}


void onReceive(int packetSize) {
    if (packetSize == 0) return; // if there's no packet, return

    if(DEBUG_MODE) Serial.println("receiving...");
    
    while (LoRa.available()) {
        // read UUID
        uint64_t UUID = 0;
        // masking ensures shift and cast don't cause problems
        UUID |= ( (uint64_t)LoRa.read() << (0 * 8) ) & 0x00000000000000FF;
        UUID |= ( (uint64_t)LoRa.read() << (1 * 8) ) & 0x000000000000FF00;
        UUID |= ( (uint64_t)LoRa.read() << (2 * 8) ) & 0x0000000000FF0000;
        UUID |= ( (uint64_t)LoRa.read() << (3 * 8) ) & 0x00000000FF000000;
        UUID |= ( (uint64_t)LoRa.read() << (4 * 8) ) & 0x000000FF00000000;
        UUID |= ( (uint64_t)LoRa.read() << (5 * 8) ) & 0x0000FF0000000000;
        UUID |= ( (uint64_t)LoRa.read() << (6 * 8) ) & 0x00FF000000000000;
        UUID |= ( (uint64_t)LoRa.read() << (7 * 8) ) & 0xFF00000000000000;

        // read partial lat
        uint16_t lat_partial = 0;
        // masking ensures shift and cast don't cause problems
        lat_partial |= ( (uint16_t)LoRa.read() << (0 * 8) ) & 0x00FF;
        lat_partial |= ( (uint16_t)LoRa.read() << (1 * 8) ) & 0xFF00;

        // read partial lat
        uint16_t lng_partial = 0;
        // masking ensures shift and cast don't cause problems
        lng_partial |= ( (uint16_t)LoRa.read() << (0 * 8) ) & 0x00FF;
        lng_partial |= ( (uint16_t)LoRa.read() << (1 * 8) ) & 0xFF00;

        if(DEBUG_MODE){
            Serial.print("received UUID ");
            Serial.println((uint32_t)UUID);
            Serial.print("received lat ");
            Serial.println(lat_partial);
            Serial.print("received lng ");
            Serial.println(lng_partial);
        }
        
        // save data
        updateBuddy(UUID, lat_partial, lng_partial);
    }
}


void sendPacket(BT_Packet packet){
    LoRa.beginPacket();
    byte *packetContents = packet.getPacket();
    LoRa.write(packetContents, PACKET_LENGTH);
    /*for(uint8_t i = 0; i < PACKET_LENGTH; i++){
        LoRa.write( *(packetContents + i) );
        if(DEBUG_MODE){
            Serial.print( *(packetContents + i) );
            Serial.print(" ");
        }
    }*/
    if(DEBUG_MODE){
        for(uint8_t i = 0; i < PACKET_LENGTH; i++){
            Serial.print(packetContents[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
    LoRa.endPacket();
}


// currently adds unknown buddies
// shows everyone
void updateBuddy(uint64_t UUID, uint16_t lat_partial, uint16_t lng_partial){
    uint8_t index = findBuddyBy(UUID);
    
    // add unknown buddies
    if(index == MAX_UINT8){
        if (buddies.size() < LARGEST_INDEX) {
          Buddy *newBuddy = new Buddy(UUID);
          buddies.add(newBuddy);
          index = buddies.size() - 1;
          // TODO might be better to get index using findBuddyBy
        }
    }

    // can't compute other Buddy's location without knowning own location
    if(myLat == LAT_LNG_ERR || myLng == LAT_LNG_ERR){
        return;
    }
    // clear 2 LSBs
    int32_t lat = myLat & 0xFFFFFFFFFFFF0000;
    int32_t lng = myLng & 0xFFFFFFFFFFFF0000;
    // replace LSBs
    lat |= lat_partial & 0x000000000000FFFF;
    lng |= lng_partial & 0x000000000000FFFF;

    Buddy *currentBuddy = buddies.get(index);
    currentBuddy->setLat(lat);
    currentBuddy->setLng(lng);

    if(DEBUG_MODE){
        Serial.print("Buddy ");
        Serial.print((uint32_t)UUID);
        Serial.print(" is at ");
        Serial.print(lat);
        Serial.print(", ");
        Serial.println(lng);
    }
}


// returns 0 if no match
uint8_t findBuddyBy(uint64_t UUID){
    for(uint8_t i = 0; i < buddies.size(); i++){
        Buddy *currentBuddy = buddies.get(i);
        if(UUID == currentBuddy->getUUID()){
            return i;
        }
    }
    return MAX_UINT8;
}


void updateUI(){
    // TODO: implement orientaion sensor to set this
    int16_t headingDegrees = 0;
    
    userInterface.clear();

    for(uint8_t i = 0; i < buddies.size(); i++){
        Buddy *currentBuddy = buddies.get(i);
        // TODO: move this into Buddy class to avoid recalculation
        int16_t currentBuddyDegrees = TinyGPSPlus::courseTo(myLat, myLng,
                currentBuddy->getLat(), currentBuddy->getLng());
        int16_t degreesDifference = headingDegrees - currentBuddyDegrees;
        while(degreesDifference > 180) degreesDifference -= 360; //correct wraparound
        if(abs(degreesDifference) > fieldOfView / 2)
            break; // out of FOV
        userInterface.setBuddyLight(degreesDifference / degreesPerLED + numLEDs / 2, buddyColor);
    }

    userInterface.show();
}


#endif
