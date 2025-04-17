#include "TheiaSerial.h"

byte DELIMITER[2] = {0xAB, 0xCD};

Stream* serial;

TheiaSerial::TheiaSerial(int rx, int tx, int board_id, int baudrate) : softwareSerial(rx, tx), boardID(board_id), useHardwareSerial(false){
    // Begin the software serial interface. This is the software constructor.
    softwareSerial.begin(baudrate);
}

TheiaSerial::TheiaSerial(int board_id,int baudrate) : boardID(board_id), useHardwareSerial(true){
    Serial.begin(baudrate);
}

void TheiaSerial::send(byte *payload, int payload_length ){
    // Start by sending the delimeter
    TheiaSerial::write_raw(DELIMITER, sizeof(DELIMITER));

    // Now send the header of the payload
    byte header[2] = {byte(TheiaSerial::boardID), byte(payload_length)};
    TheiaSerial::write_raw(header, 2);

    // Now send the payload
    TheiaSerial::write_raw(payload, payload_length);
}

packet TheiaSerial::read(int timeout){
    // Allocate memory & initialize with error packet
    byte header[2], delimeter[2];
    packet return_packet;
    int time_elapsed = 0;

    // Add the timeout
    while( !available() && time_elapsed <= timeout ){
        delay(10);
        time_elapsed += 10;
    }

    // Send error packet if timeout
    if(time_elapsed >= timeout){
        return_packet.id = -1;
        return return_packet;
    }

    // Get the delimiter
    read_bytes_raw(delimeter, 2);

    // Send error packet if bad delimeter
    if (delimeter[0] != 0xAB || delimeter[1] != 0xCD){
        return_packet.id = -2;
        return return_packet;
    }

    // Get the header
    read_bytes_raw(header, 2);
    return_packet.id = int(header[0]);
    return_packet.payload_length = int(header[1]);

    byte payload[return_packet.payload_length];
    read_bytes_raw(payload, return_packet.payload_length);
    return_packet.payload = payload;

    return return_packet;
}

/// @brief The private method to write to whatever serial we are currently using
/// @param payload The payload to send
/// @param payload_length The length of the payload being sent

void TheiaSerial::write_raw(byte *payload, int payload_length){
    if( useHardwareSerial ){
        Serial.write(payload, payload_length);
    }else{
        softwareSerial.write(payload, payload_length);
    }
}

void TheiaSerial::read_bytes_raw(byte* buffer, int num_bytes){
    byte incoming_packet[num_bytes];
    if( useHardwareSerial ){
        Serial.readBytes(buffer, num_bytes);
    }else{
        softwareSerial.readBytes(buffer, num_bytes);
    }
}

bool TheiaSerial::available(){
    if( useHardwareSerial ){
        return Serial.available();
    }else{
        return softwareSerial.available();
    }
}