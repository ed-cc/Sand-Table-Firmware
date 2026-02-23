// move_command.h - Move command types for the motion pipeline
#ifndef MOVE_COMMAND_H
#define MOVE_COMMAND_H

#include <Arduino.h>

enum MoveType : uint8_t {
    MOVE_NONE = 0,
    MOVE_TO,
    MOVE_RELATIVE,
    MOVE_HOME,
    MOVE_SET_SPEED,
    MOVE_SET_POSITION,
    MOVE_STOP,
    MOVE_ENABLE,
    MOVE_DISABLE
};

struct MoveCommand {
    MoveType type;
    float radius;
    float theta;
};

#endif // MOVE_COMMAND_H
