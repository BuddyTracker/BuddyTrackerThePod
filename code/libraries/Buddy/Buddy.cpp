#include <Arduino.h>
#include "Buddy.h"

Buddy::Buddy(){
    // error/default value
    this->UUID = 0;
}

Buddy::Buddy(uint64_t UUID){
    this->UUID = UUID;
}

uint64_t Buddy::getUUID(void){
    return UUID;
}

uint32_t Buddy::getLat(void){
    return lat;
}

uint32_t Buddy::getLng(void){
    return lng;
}

void Buddy::setLat(uint32_t lat){
    this->lat = lat;
}

void Buddy::setLng(uint32_t lng){
    this->lng = lng;
}
