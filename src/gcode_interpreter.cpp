// gcode_interpreter.cpp - State machine GCode parser
#include "gcode_interpreter.h"
#include <math.h>

GCodeInterpreter::GCodeInterpreter(
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& gcodeBuffer,
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& moveBuffer,
    StatusCallback onStatusQuery)
    : m_gcodeBuffer(gcodeBuffer)
    , m_moveBuffer(moveBuffer)
    , m_onStatusQuery(onStatusQuery)
    , m_absoluteMode(true)
    , m_activeMotion(0)
    , m_feedrate(0)
    , m_state(INTERP_IDLE)
{}

bool GCodeInterpreter::run() {
    switch (m_state) {
        case INTERP_IDLE:
            if (m_gcodeBuffer.isEmpty()) return false;
            beginLine();
            return true;

        case INTERP_PARSING:
            parseNextToken();
            return true;

        case INTERP_EXECUTE:
            if (m_moveBuffer.size() >= MOVE_BUFFER_CAPACITY - 1) return false;
            executeLine();
            return true;
    }
    return false;
}

void GCodeInterpreter::beginLine() {
    GCodeLine gcode;
    m_gcodeBuffer.dequeue(gcode);
    memcpy(m_line, gcode.line, GCODE_LINE_MAX_LENGTH);

    // Strip comment
    char* comment = strchr(m_line, ';');
    if (comment) *comment = '\0';

    // Reset parse state
    m_parsePos = 0;
    m_gCount = 0;
    m_mCode = -1;
    m_rVal = NAN;
    m_aVal = NAN;
    m_fVal = NAN;
    m_hasR = false;
    m_hasA = false;

    m_state = INTERP_PARSING;
}

void GCodeInterpreter::parseNextToken() {
    char* p = m_line + m_parsePos;

    // Skip whitespace
    while (*p == ' ' || *p == '\t') p++;

    // End of line — move to execute
    if (!*p) {
        m_state = INTERP_EXECUTE;
        return;
    }

    // Read letter
    char letter = *p;
    if (letter >= 'a' && letter <= 'z') letter -= 32;
    if (letter < 'A' || letter > 'Z') {
        // Not a letter — skip and save position for next call
        m_parsePos = (p + 1) - m_line;
        return;
    }
    p++;

    // Parse number (this is the expensive strtod call, one per run())
    char* end;
    float value = (float)strtod(p, &end);
    if (end == p) {
        // Letter with no number (e.g. bare "R" or "A" in G28 R A)
        m_parsePos = p - m_line;
        switch (letter) {
            case 'R': m_hasR = true; m_rVal = 0; break;
            case 'A': m_hasA = true; m_aVal = 0; break;
        }
        return;
    }
    p = end;

    // Store parsed token
    switch (letter) {
        case 'G':
            if (m_gCount < MAX_GCODES_PER_LINE)
                m_gCodes[m_gCount++] = (int8_t)value;
            break;
        case 'M': m_mCode = (int8_t)value; break;
        case 'R': m_rVal = value; m_hasR = true; break;
        case 'A': m_aVal = value; m_hasA = true; break;
        case 'F': m_fVal = value; break;
    }

    m_parsePos = p - m_line;
}

void GCodeInterpreter::executeLine() {
    // Apply feedrate if changed
    if (!isnan(m_fVal) && m_fVal != m_feedrate) {
        m_feedrate = m_fVal;
        MoveCommand cmd = {MOVE_SET_SPEED, m_feedrate, 0};
        m_moveBuffer.enqueue(cmd);
    }

    // Process G codes in order (handles "G90 G28" on one line)
    bool handled = false;
    for (uint8_t i = 0; i < m_gCount; i++) {
        handled = true;
        switch (m_gCodes[i]) {
            case 0:
            case 1:
                m_activeMotion = m_gCodes[i];
                if (m_hasR || m_hasA) enqueueMove(m_hasR, m_rVal, m_hasA, m_aVal);
                break;
            case 28: {
                MoveCommand cmd = {MOVE_HOME, 0, 0};
                m_moveBuffer.enqueue(cmd);
                break;
            }
            case 90: m_absoluteMode = true; break;
            case 91: m_absoluteMode = false; break;
            case 92: {
                MoveCommand cmd = {MOVE_SET_POSITION,
                    m_hasR ? m_rVal : NAN, m_hasA ? m_aVal : NAN};
                m_moveBuffer.enqueue(cmd);
                break;
            }
            default:
                Serial.print(F("error: Unknown command G"));
                Serial.println(m_gCodes[i]);
                m_state = INTERP_IDLE;
                return;
        }
    }

    // M code
    if (m_mCode >= 0) {
        handled = true;
        switch (m_mCode) {
            case 0:
            case 2: {
                MoveCommand cmd = {MOVE_STOP, 0, 0};
                m_moveBuffer.enqueue(cmd);
                break;
            }
            case 17: {
                MoveCommand cmd = {MOVE_ENABLE, 1, 1};
                m_moveBuffer.enqueue(cmd);
                break;
            }
            case 18: {
                MoveCommand cmd = {MOVE_DISABLE, 1, 1};
                m_moveBuffer.enqueue(cmd);
                break;
            }
            case 114: {
                MachineStatus status;
                m_onStatusQuery(status);
                Serial.print(F("R:"));
                Serial.print(status.radius, 2);
                Serial.print(F(" A:"));
                Serial.print(status.theta, 2);
                Serial.print(F(" F:"));
                Serial.println(status.feedrate, 0);
                break;
            }
            case 115:
                Serial.println(F("FIRMWARE:SandTable VERSION:1.0.0"));
                break;
            default:
                Serial.print(F("error: Unknown command M"));
                Serial.println(m_mCode);
                m_state = INTERP_IDLE;
                return;
        }
    }

    // Modal motion: bare R/A/F parameters use active motion mode
    if (!handled && (m_hasR || m_hasA)) {
        enqueueMove(m_hasR, m_rVal, m_hasA, m_aVal);
        handled = true;
    }

    if (handled || !isnan(m_fVal)) {
        Serial.println(F("ok"));
    }

    m_state = INTERP_IDLE;
}

void GCodeInterpreter::enqueueMove(bool hasR, float r, bool hasA, float a) {
    MoveType type = m_absoluteMode ? MOVE_TO : MOVE_RELATIVE;
    MoveCommand cmd = {type,
        hasR ? r : (m_absoluteMode ? NAN : 0),
        hasA ? a : (m_absoluteMode ? NAN : 0)};
    m_moveBuffer.enqueue(cmd);
}
