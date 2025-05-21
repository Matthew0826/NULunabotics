#ifndef THEIA_SERIAL_H
#define THEIA_SERIAL_H
#include "Hashtable.h"
#include <Arduino.h>

class TheiaSerial {
private:
    typedef void (*ParserFunc)(const uint8_t* data, size_t len);

    struct HandlerEntry {
        ParserFunc parser;
        size_t structSize;
    };

    // Serial buffer parameters
    static const size_t READ_BUFFER_SIZE = 256;
    static inline uint8_t readBuffer[READ_BUFFER_SIZE];
    static inline size_t readBufferHead = 0;
    static inline size_t readBufferTail = 0;

    // Registry: pub_id -> handler
    static inline Hashtable<int, HandlerEntry>& registry() {
        static Hashtable<int, HandlerEntry> reg;
        return reg;
    }

    // Per-type handler storage (map pub_id -> handler function)
    template<typename T>
    static inline Hashtable<int, void(*)(const T&)>& handlerMap() {
        static Hashtable<int, void(*)(const T&)> handlers;
        return handlers;
    }

    // For delayed announcement
    static inline bool& initialized() {
        static bool value = false;
        return value;
    }

    static inline unsigned long& initTime() {
        static unsigned long t = 0;
        return t;
    }

    // Helper to read a byte from our buffer
    static bool readByte(uint8_t& byteOut) {
        if (readBufferHead == readBufferTail) {
            return false; // Buffer is empty
        }
        byteOut = readBuffer[readBufferHead];
        readBufferHead = (readBufferHead + 1) % READ_BUFFER_SIZE;
        return true;
    }

    // Fill the buffer with available Serial data
    static void fillBuffer() {
        while (Serial.available() > 0) {
            // Calculate next position after tail
            size_t nextTail = (readBufferTail + 1) % READ_BUFFER_SIZE;
            
            // Check if buffer is full
            if (nextTail == readBufferHead) {
                break; // Buffer full
            }
            
            // Read byte and store it
            readBuffer[readBufferTail] = Serial.read();
            readBufferTail = nextTail;
        }
    }

public:
    // Static trampoline parser: looks up real handler and calls it
    template<typename T>
    static void genericParser(const uint8_t* data, size_t len) {
        if (len < sizeof(T)) return;
        T obj;
        memcpy(&obj, data, sizeof(T));

        auto& map = handlerMap<T>();
        
        for (int id : map.keys()) {
            auto handlerPtr = map.get(id);
            if (handlerPtr) {
                (*handlerPtr)(obj);
            }
        }
    }

    static void begin() {
        Serial.begin(19200);
        while (!Serial) {
            delay(10);
        }
        // Initialize buffer pointers
        readBufferHead = 0;
        readBufferTail = 0;
    }

    // Register an ID and its handler
    template <typename T>
    static void addId(int pubId, void (*handler)(const T&)) {
        HandlerEntry entry;
        entry.parser = &genericParser<T>;
        entry.structSize = sizeof(T);

        registry().put(pubId, entry);
        handlerMap<T>().put(pubId, handler);
    }

    static void sendHeader(int pubId, int payloadLen) {
        const uint8_t header[] = {0xAB, 0xCD};
        Serial.write(header, 2);
        Serial.write(reinterpret_cast<const uint8_t*>(&pubId), 1);
        Serial.write(reinterpret_cast<const uint8_t*>(&payloadLen), 2);
    }

    // Send a struct with framing
    template <typename T>
    static void sendFramedMessage(int pubId, const T& data) {
        sendHeader(pubId, sizeof(T));
        Serial.write(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
        // Wait for transmission to complete
        Serial.flush();
    }

    // Deserialize payload into struct
    template <typename T>
    static bool parseFramedMessage(const uint8_t* buffer, size_t buffer_size, T& out_struct) {
        if (buffer_size < sizeof(T)) return false;
        memcpy(&out_struct, buffer, sizeof(T));
        return true;
    }

    // Tell the Raspberry Pi all of the ids used by this board
    static void broadcastIds() {
        auto& map = registry();
        for (int id : map.keys()) {
            auto entry = map.get(id);
            size_t payloadLen = entry->structSize;
            sendHeader(id, payloadLen);
            delay(10);
            // Write a blank message
            for (int i = 0; i < payloadLen; i++) {
                Serial.write(0x00);
            }
        }
    }

    // Read and dispatch incoming messages
    static void tick() {
        // First, fill our buffer with any available data
        fillBuffer();

        static enum {
            WAIT_HEADER_1,
            WAIT_HEADER_2,
            READ_PUB_ID,
            READ_LEN_1,
            READ_LEN_2,
            READ_PAYLOAD
        } state = WAIT_HEADER_1;

        static uint8_t pubId = 0;
        static uint16_t payloadLength = 0;
        static uint16_t payloadIndex = 0;
        static uint8_t payloadBuffer[512];
        static unsigned long lastByteTime = 0;
        
        // Handle timeout to recover from incomplete message reception
        if (state != WAIT_HEADER_1 && millis() - lastByteTime > 100) {
            state = WAIT_HEADER_1; // Reset state if stuck waiting for bytes
        }

        uint8_t byteIn;
        while (readByte(byteIn)) {  // Process buffered bytes
            lastByteTime = millis();

            switch (state) {
                case WAIT_HEADER_1:
                    if (byteIn == 0xAB) state = WAIT_HEADER_2;
                    // else return; // Invalid start byte
                    break;

                case WAIT_HEADER_2:
                    state = (byteIn == 0xCD) ? READ_PUB_ID : WAIT_HEADER_1;
                    // if (byteIn != 0xCD) return;
                    break;

                case READ_PUB_ID:
                    pubId = byteIn;
                    // Check if byte represents a "re-broadcast of ids" request
                    if (byteIn == 0xFF) {
                        broadcastIds();
                        state = WAIT_HEADER_1;
                        // return;
                    // Check if the id was registered for this board
                    } else if (registry().exists((int)pubId)) {
                        state = READ_LEN_1;
                    // So it must be an unknown id
                    } else {
                        state = WAIT_HEADER_1;
                        // return;
                    }
                    break;

                case READ_LEN_1:
                    payloadLength = byteIn << 8;
                    state = READ_LEN_2;
                    break;

                case READ_LEN_2:
                    payloadLength |= byteIn;
                    if (payloadLength > sizeof(payloadBuffer)) {
                        state = WAIT_HEADER_1;  // payload too big
                        // return;
                    } else {
                        payloadIndex = 0;
                        state = READ_PAYLOAD;
                    }
                    break;

                case READ_PAYLOAD:
                    payloadBuffer[payloadIndex++] = byteIn;
                    if (payloadIndex >= payloadLength) {
                        auto entry = registry().get((int)pubId);
                        entry->parser(payloadBuffer, payloadLength);
                        state = WAIT_HEADER_1;
                        // return;
                    }
                    break;
            }
        }
    }

    // Get the number of bytes available in our buffer
    static size_t available() {
        if (readBufferTail >= readBufferHead) {
            return readBufferTail - readBufferHead;
        } else {
            return READ_BUFFER_SIZE - readBufferHead + readBufferTail;
        }
    }
};

#endif
