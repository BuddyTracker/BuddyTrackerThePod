#include <Arduino.h>
#include "BT_Packet.h"

BT_Packet::BT_Packet(uint64_t UUID, int32_t lat, int32_t lng){
    this->UUID = UUID;
    this->lat = lat;
    this->lng = lng;
    updatePending = true;
}

void BT_Packet::setGPS(int32_t lat, int32_t lng){
    this->lat = lat;
    this->lng = lng;
    updatePending = true;
}

bool BT_Packet::updatesPending(){
    return updatePending;
}

byte *BT_Packet::getPacket(){
    updatePending = false;

    byte *packet = new byte[PACKET_LENGTH];
    for(int i = 0; i < 8; i++){
        packet[i] = (uint8_t)(UUID >> (8 * i)) % 0xFF;
    }
    packet[9] = (uint8_t) ( (lat && 0x0000FF00) >> 8 );
    packet[10] = (uint8_t)(lat && 0x000000FF);
    packet[11] = (uint8_t) ( (lng && 0x0000FF00) >> 8 );
    packet[12] = (uint8_t)(lng && 0x000000FF);
    return packet;
}