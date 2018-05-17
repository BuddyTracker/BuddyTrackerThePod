#include <Adafruit_GPS.h>
#include <Arduino.h>
#include <LinkedList.h>
#include <LoRa.h>
#include <SPI.h>
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
//void startGPS();
//void handleGPS();
uint8_t findBuddyBy(uint64_t UUID);


// TODO: define comparator for sorting LinkedList
LinkedList<Buddy*> buddies;

uint32_t timer = millis();
const int32_t LAT_LNG_ERR = 999999999;

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
    
    // TODO: only sending if updatesPending (or at least adjust timing)
    //sendPacket(myPacket);

    //delay(3000);
}


void onReceive(int packetSize) {
    if (packetSize == 0) return; // if there's no packet, return

    if(DEBUG_MODE)Serial.println("receiving...");
    
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


/*void startGPS(){
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz
        
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);
    
    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);
}*/


// TODO: this is largely derived from example code, ensure it works
// TODO: maybe handle with interupt instead?
/*void handleGPS(){
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
    }
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis()) timer = millis();
        
    // approximately every 2 seconds or so, update lat and lng values
    if (millis() - timer > 2000) {
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        if (GPS.fix) {
            myLat = GPS.latitude;
            myLng = GPS.longitude;
            Serial.print("Location: ");
            Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        }
    }
}*/


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


#endif
