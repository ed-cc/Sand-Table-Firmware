// serial_reader.h - Non-blocking serial line reader
#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include <Arduino.h>
#include "ring_buffer.h"

#define GCODE_LINE_MAX_LENGTH 64
#define GCODE_BUFFER_CAPACITY 8

struct GCodeLine {
    char line[GCODE_LINE_MAX_LENGTH];
};

typedef void (*EmergencyStopCallback)();

struct MachineStatus {
    float radius;
    float theta;
    float feedrate;
    bool isMoving;
};
typedef void (*StatusCallback)(MachineStatus& status);
typedef void (*ResumeCallback)();

class SerialReader {
public:
    SerialReader(RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& buffer,
                 EmergencyStopCallback onEmergencyStop,
                 StatusCallback onStatusQuery,
                 ResumeCallback onResume)
        : m_buffer(buffer)
        , m_onEmergencyStop(onEmergencyStop)
        , m_onStatusQuery(onStatusQuery)
        , m_onResume(onResume)
        , m_pos(0) {}

    // Call once per loop iteration. Reads at most one character from serial
    // and enqueues a complete line when a newline is received.
    // Returns true if a new line was enqueued this call.
    bool run() {
        if (!Serial.available()) {
            return false;
        }

        char c = Serial.read();

        // Real-time commands - immediate, no line buffering
        if (c == '!') {
            m_pos = 0;
            m_onEmergencyStop();
            return false;
        }

        if (c == '?') {
            MachineStatus status;
            m_onStatusQuery(status);
            Serial.print(status.isMoving ? F("<Run|R:") : F("<Idle|R:"));
            Serial.print(status.radius, 2);
            Serial.print(F(",A:"));
            Serial.print(status.theta, 2);
            Serial.print(F(",F:"));
            Serial.print(status.feedrate, 0);
            Serial.println(F(">"));
            return false;
        }

        if (c == '~') {
            m_onResume();
            return false;
        }

        // Ignore carriage returns
        if (c == '\r') {
            return false;
        }

        // Newline terminates the line
        if (c == '\n') {
            if (m_pos == 0) {
                return false;  // Skip empty lines
            }
            m_lineBuffer[m_pos] = '\0';

            GCodeLine gcode;
            memcpy(gcode.line, m_lineBuffer, m_pos + 1);
            m_pos = 0;
            return m_buffer.enqueue(gcode);
        }

        // Accumulate character, silently drop if line is too long
        if (m_pos < GCODE_LINE_MAX_LENGTH - 1) {
            m_lineBuffer[m_pos++] = c;
        }

        return false;
    }

private:
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& m_buffer;
    EmergencyStopCallback m_onEmergencyStop;
    StatusCallback m_onStatusQuery;
    ResumeCallback m_onResume;
    char m_lineBuffer[GCODE_LINE_MAX_LENGTH];
    uint8_t m_pos;
};

#endif // SERIAL_READER_H
