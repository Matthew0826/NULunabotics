#ifndef THEIA_SERIAL_H
#define THEIA_SERIAL_H

#include <Arduino.h>
#include <SoftwareSerial.h>

typedef struct{
    int id;
    int payload_length;
    byte* payload;
} packet;


class TheiaSerial{
public:
    TheiaSerial(int rx , int tx, int board_id, int baudrate = 9600);
    TheiaSerial(int board_id, int baudrate = 9600);

    void send(byte *payload, int payload_length);
    packet read(int timeout = 1000);
    
private:
    SoftwareSerial softwareSerial;
    bool useHardwareSerial;
    int boardID;

    void write_raw(byte *payload, int payload_length);
    void read_bytes_raw(byte *buffer, int num_bytes);
    bool available();

};

#endif