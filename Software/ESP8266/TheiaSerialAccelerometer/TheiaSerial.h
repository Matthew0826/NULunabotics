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

public:
    // Static trampoline parser: looks up real handler and calls it
    template<typename T>
    static void genericParser(const uint8_t* data, size_t len) {
        if (len < sizeof(T)) return;
        T obj;
        memcpy(&obj, data, sizeof(T));

        auto& map = handlerMap<T>();
        auto keysVector = map.keys();
        // for (unsigned int i = 0; i < keysVector.elements(); i++) {
        //     int id = keysVector.get(i);
        //     auto entry = registry().get(id);
        //     if (entry && entry->parser == genericParser<T>) {
        //         auto handlerPtr = map.get(id);
        //         if (handlerPtr) (*handlerPtr)(obj);
        //         return;
        //     }
        // }
        
        for (int id : map.keys()) {
            auto handlerPtr = map.get(id);
            if (handlerPtr) {
                (*handlerPtr)(obj);
            }
        }
    }

    static void begin() {
        Serial.begin(9600);
        while (!Serial) {
            delay(10);
        }
    }

    // Register an ID and its handler
    template <typename T>
    static void addId(int pub_id, void (*handler)(const T&)) {
        HandlerEntry entry;
        entry.parser = &genericParser<T>;
        entry.structSize = sizeof(T);
        registry().put(pub_id, entry);
        
        handlerMap<T>().put(pub_id, handler);

        delay(1000);
        T empty{};
        sendFramedMessage(pub_id, empty);
    }

    // Send a struct with framing
    template <typename T>
    static void sendFramedMessage(int pub_id, const T& data) {
        const uint8_t header[] = {0xAB, 0xCD};
        Serial.write(header, 2);
        Serial.write(reinterpret_cast<const uint8_t*>(&pub_id), 1);

        uint16_t payload_len = sizeof(T);
        Serial.write(reinterpret_cast<const uint8_t*>(&payload_len), 2);
        Serial.write(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
    }

    // Deserialize payload into struct
    template <typename T>
    static bool parseFramedMessage(const uint8_t* buffer, size_t buffer_size, T& out_struct) {
        if (buffer_size < sizeof(T)) return false;
        memcpy(&out_struct, buffer, sizeof(T));
        return true;
    }

    // Read and dispatch incoming messages
    static void tick() {
        static enum {
            WAIT_HEADER_1,
            WAIT_HEADER_2,
            READ_PUB_ID,
            READ_LEN_1,
            READ_LEN_2,
            READ_PAYLOAD
        } state = WAIT_HEADER_1;

        static uint8_t pub_id = 0;
        static uint16_t payload_len = 0;
        static uint16_t payload_index = 0;
        static uint8_t payload_buffer[256];

        while (Serial.available()) {
            uint8_t byte_in = Serial.read();

            switch (state) {
                case WAIT_HEADER_1:
                    if (byte_in == 0xAB) state = WAIT_HEADER_2;
                    break;

                case WAIT_HEADER_2:
                    state = (byte_in == 0xCD) ? READ_PUB_ID : WAIT_HEADER_1;
                    break;

                case READ_PUB_ID:
                    pub_id = byte_in;
                    if (registry().exists((int)pub_id)) {
                        state = READ_LEN_1;
                    } else {
                        state = WAIT_HEADER_1;  // unknown ID
                    }
                    break;

                case READ_LEN_1:
                    payload_len = byte_in << 8;
                    state = READ_LEN_2;
                    break;

                case READ_LEN_2:
                    payload_len |= byte_in;
                    if (payload_len > sizeof(payload_buffer)) {
                        state = WAIT_HEADER_1;  // too big
                    } else {
                        payload_index = 0;
                        state = READ_PAYLOAD;
                    }
                    break;

                case READ_PAYLOAD:
                    payload_buffer[payload_index++] = byte_in;
                    if (payload_index >= payload_len) {
                        auto entry = registry().get(pub_id);
                        entry->parser(payload_buffer, payload_len);
                        state = WAIT_HEADER_1;
                    }
                    break;
            }
        }
    }
};

#endif
