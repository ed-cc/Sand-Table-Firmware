// motion_commander.h - Dequeues MoveCommands and dispatches them to Motion
#ifndef MOTION_COMMANDER_H
#define MOTION_COMMANDER_H

#include <Arduino.h>
#include "ring_buffer.h"
#include "move_command.h"
#include "motion.h"
#include "gcode_interpreter.h"

class MotionCommander {
public:
    MotionCommander(
        RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& moveBuffer,
        Motion& motion);

    // Call every event loop iteration.
    // Dequeues and dispatches the next command when motion is idle.
    // Returns true if a command was dispatched this call.
    bool run();

private:
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& m_moveBuffer;
    Motion& m_motion;

    MoveCommand m_pending;
    bool m_hasPending;

    bool tryDispatch(const MoveCommand& cmd);
};

#endif // MOTION_COMMANDER_H
