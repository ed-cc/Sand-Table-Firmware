// gcode_interpreter.h - Converts GCodeLines into MoveCommands
//
// Interpretation is spread across multiple run() calls via a state machine.
// Each call does at most one strtod (~50-100μs on AVR), keeping the
// event loop responsive for stepper timing.
//
#ifndef GCODE_INTERPRETER_H
#define GCODE_INTERPRETER_H

#include <Arduino.h>
#include "ring_buffer.h"
#include "move_command.h"
#include "serial_reader.h"

#define MOVE_BUFFER_CAPACITY 32
#define MAX_GCODES_PER_LINE 3

enum InterpreterState : uint8_t {
    INTERP_IDLE,
    INTERP_PARSING,
    INTERP_EXECUTE,
};

class GCodeInterpreter {
public:
    GCodeInterpreter(
        RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& gcodeBuffer,
        RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& moveBuffer,
        StatusCallback onStatusQuery);

    // Call every event loop iteration. Advances interpretation by one step:
    //   IDLE    → dequeue a line, strip comments, reset parse state
    //   PARSING → parse one letter+number token (one strtod at most)
    //   EXECUTE → generate MoveCommands from parsed tokens, send ok/error
    // Returns true if work was done this call.
    bool run();

private:
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& m_gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& m_moveBuffer;
    StatusCallback m_onStatusQuery;

    // Modal state (persists across lines)
    bool m_absoluteMode;
    uint8_t m_activeMotion;  // 0 = G0, 1 = G1
    float m_feedrate;

    // State machine
    InterpreterState m_state;

    // Line being parsed
    char m_line[GCODE_LINE_MAX_LENGTH];
    uint8_t m_parsePos;

    // Accumulated parse results for current line
    int8_t m_gCodes[MAX_GCODES_PER_LINE];
    uint8_t m_gCount;
    int8_t m_mCode;
    float m_rVal, m_aVal, m_fVal;
    bool m_hasR, m_hasA;

    void beginLine();
    void parseNextToken();
    void executeLine();
    void enqueueMove(bool hasR, float r, bool hasA, float a);
};

#endif // GCODE_INTERPRETER_H
