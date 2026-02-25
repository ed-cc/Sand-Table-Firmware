// subprogram_runner.cpp - PROGMEM subprogram line feeder
#include "subprogram_runner.h"

SubprogramRunner::SubprogramRunner(
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& gcodeBuffer)
    : m_gcodeBuffer(gcodeBuffer)
    , m_active(false)
    , m_linesPtr(nullptr)
    , m_lineCount(0)
    , m_lineIndex(0)
    , m_repsRemaining(0)
{}

bool SubprogramRunner::start(uint8_t programNumber, uint16_t repetitions) {
    const Subprogram* prog = findSubprogram(programNumber);
    if (!prog) return false;

    // Cache PROGMEM fields into SRAM for fast access during run()
    m_linesPtr  = (const char* const*)pgm_read_ptr(&prog->lines);
    m_lineCount = pgm_read_byte(&prog->lineCount);
    m_lineIndex = 0;
    m_repsRemaining = repetitions;
    m_active = true;

    Serial.print(F("Subprogram P"));
    Serial.print(programNumber);
    Serial.print(F(" x"));
    Serial.println(repetitions);
    return true;
}

bool SubprogramRunner::run() {
    if (!m_active) return false;
    if (m_gcodeBuffer.isFull()) return false;

    // Read one line from PROGMEM and enqueue it
    GCodeLine gcode;
    PGM_P pgmLine = (PGM_P)pgm_read_ptr(&m_linesPtr[m_lineIndex]);
    strncpy_P(gcode.line, pgmLine, GCODE_LINE_MAX_LENGTH - 1);
    gcode.line[GCODE_LINE_MAX_LENGTH - 1] = '\0';

    m_gcodeBuffer.enqueue(gcode);
    m_lineIndex++;

    // Check if we finished one repetition
    if (m_lineIndex >= m_lineCount) {
        m_repsRemaining--;
        if (m_repsRemaining == 0) {
            m_active = false;
            Serial.println(F("Subprogram complete"));
        } else {
            m_lineIndex = 0;  // restart for next repetition
        }
    }

    return true;
}

void SubprogramRunner::abort() {
    m_active = false;
    m_lineIndex = 0;
    m_repsRemaining = 0;
}
