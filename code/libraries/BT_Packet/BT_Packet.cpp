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
    
    byte packet[PACKET_LENGTH];
    for(int i = 0; i < 8; i++){
        packet[i] = (UUID >> (8 * i)) % 0xff;
    }
    packet[9] = lat >> 8;
    packet[10] = lat % 0xff;
    packet[11] = lng >> 8;
    packet[12] = lng % 0xff;
    return packet;
}