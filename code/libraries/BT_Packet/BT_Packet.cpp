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

//TODO: look into union + struct http://www.cplusplus.com/doc/tutorial/other_data_types/
uint8_t *BT_Packet::getPacket(){
    updatePending = false;

    //TODO: C++ memset, set packet to 0s
    for(int i = 0; i < 8; i++){
        packet[i] = (uint8_t)(UUID >> (8 * i)) & 0x000000FF;
    }
    packet[8] = (uint8_t) ( (lat & 0x0000FF00) >> 8 );
    packet[9] = (uint8_t)(lat & 0x000000FF);
    packet[10] = (uint8_t) ( (lng & 0x0000FF00) >> 8 );
    packet[11] = (uint8_t)(lng & 0x000000FF);
    return packet;
}