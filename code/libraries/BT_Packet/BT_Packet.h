#ifndef BT_PACKET_H
#define BT_PACKET_H


#define PACKET_LENGTH 12


class BT_Packet {
    public:
        BT_Packet(uint64_t UUID);
        
        void setGPS(int32_t lat, int32_t lng);
        bool updatesPending();
        byte *getPacket();
    protected:
        uint64_t UUID;
        int32_t lat;
        int32_t lng;
        bool updatePending;
};


#endif