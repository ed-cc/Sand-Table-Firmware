// subprogram_runner.h - Feeds PROGMEM-stored G-code into the pipeline
//
// When activated via M98, the runner copies one G-code line per run() call
// from flash into the gcodeBuffer. This reuses the entire existing parsing
// and motion pipeline with zero changes to the interpreter internals.
//
#ifndef SUBPROGRAM_RUNNER_H
#define SUBPROGRAM_RUNNER_H

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "ring_buffer.h"
#include "serial_reader.h"          // GCodeLine, GCODE_BUFFER_CAPACITY
#include "subprogram_library.h"     // Subprogram, findSubprogram

class SubprogramRunner {
public:
    SubprogramRunner(RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& gcodeBuffer);

    // Start running subprogram with given P number, repeated L times.
    // Returns true if the program was found and started.
    bool start(uint8_t programNumber, uint16_t repetitions);

    // Call every event loop iteration.
    // Copies one PROGMEM line into gcodeBuffer if space is available.
    // Returns true if a line was enqueued this call.
    bool run();

    // True when a subprogram is actively feeding lines
    bool isActive() const { return m_active; }

    // Immediately stop the current subprogram
    void abort();

private:
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& m_gcodeBuffer;

    bool m_active;
    const char* const* m_linesPtr;  // cached PROGMEM pointer to lines array
    uint8_t m_lineCount;            // cached from PROGMEM descriptor
    uint8_t m_lineIndex;            // current line within one repetition
    uint16_t m_repsRemaining;       // repetitions left (including current)
};

#endif // SUBPROGRAM_RUNNER_H
