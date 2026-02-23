// test_gcode_interpreter.cpp - Unit tests for GCodeInterpreter
#include <unity.h>
#include "gcode_interpreter.h"

// Mock globals
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// Status callback state
static bool statusQueryCalled = false;
static void mockStatusQuery(MachineStatus& status) {
    statusQueryCalled = true;
    status.radius = 25.0f;
    status.theta = 90.0f;
    status.feedrate = 500.0f;
    status.isMoving = true;
}

// Helper: feed a gcode line string into the interpreter and run until idle
static void interpret(GCodeInterpreter& interp,
                      RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY>& gcodeBuffer,
                      const char* line)
{
    GCodeLine gcode;
    strncpy(gcode.line, line, GCODE_LINE_MAX_LENGTH - 1);
    gcode.line[GCODE_LINE_MAX_LENGTH - 1] = '\0';
    gcodeBuffer.enqueue(gcode);

    // Run until interpreter returns to idle (max iterations as safety)
    for (int i = 0; i < 200; i++) {
        if (!interp.run()) break;
    }
}

static void resetTestState(RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& moveBuffer) {
    // Drain move buffer
    MoveCommand cmd;
    while (moveBuffer.dequeue(cmd)) {}
    Serial.clearOutput();
    statusQueryCalled = false;
}

// ===== Test Cases =====

void test_g1_absolute_move() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 R50 A120");

    TEST_ASSERT_EQUAL(1, moveBuffer.size());
    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, cmd.theta);
}

void test_g0_rapid_move() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G0 R100 A0");

    TEST_ASSERT_EQUAL(1, moveBuffer.size());
    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, cmd.theta);
}

void test_g1_radius_only_theta_nan() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 R30");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, cmd.radius);
    TEST_ASSERT_TRUE(isnan(cmd.theta));
}

void test_g1_theta_only_radius_nan() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 A90");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_TRUE(isnan(cmd.radius));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, cmd.theta);
}

void test_g91_relative_mode() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G91");
    Serial.clearOutput();

    interpret(interp, gcodeBuffer, "G1 R10 A20");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_RELATIVE, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, cmd.theta);
}

void test_g90_absolute_mode() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    // Switch to relative, then back to absolute
    interpret(interp, gcodeBuffer, "G91");
    interpret(interp, gcodeBuffer, "G90");
    resetTestState(moveBuffer);

    interpret(interp, gcodeBuffer, "G1 R10 A20");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
}

void test_g91_omitted_axis_zero() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G91");
    resetTestState(moveBuffer);

    // In relative mode, omitted axis should be 0 (not NAN)
    interpret(interp, gcodeBuffer, "G1 R10");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_RELATIVE, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, cmd.theta);
}

void test_g28_home() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G28");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_HOME, cmd.type);
}

void test_g92_set_position() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G92 R0 A0");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_SET_POSITION, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, cmd.theta);
}

void test_feedrate_emits_set_speed() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 R10 A20 F1000");

    // Should produce SET_SPEED then MOVE_TO
    TEST_ASSERT_EQUAL(2, moveBuffer.size());

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_SET_SPEED, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000.0f, cmd.radius);

    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
}

void test_feedrate_not_repeated() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 R10 F1000");
    resetTestState(moveBuffer);

    // Same feedrate again — should NOT emit another SET_SPEED
    interpret(interp, gcodeBuffer, "G1 R20 F1000");

    TEST_ASSERT_EQUAL(1, moveBuffer.size());
    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
}

void test_modal_motion_bare_coordinates() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    // Set active motion mode with G1
    interpret(interp, gcodeBuffer, "G1 R10 A20");
    resetTestState(moveBuffer);

    // Bare coordinates should use active motion mode (G1 = MOVE_TO)
    interpret(interp, gcodeBuffer, "R30 A40");

    TEST_ASSERT_EQUAL(1, moveBuffer.size());
    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, cmd.theta);
}

void test_comment_stripping() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 R10 A20 ; this is a comment");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, cmd.theta);
}

void test_multiple_gcodes_on_one_line() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    // G90 + G1 on same line
    interpret(interp, gcodeBuffer, "G90 G1 R10 A20");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, cmd.radius);
}

void test_m0_stop() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "M0");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_STOP, cmd.type);
}

void test_m2_stop() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "M2");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_STOP, cmd.type);
}

void test_m114_status_report() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    Serial.clearOutput();
    interpret(interp, gcodeBuffer, "M114");

    TEST_ASSERT_TRUE(statusQueryCalled);
    TEST_ASSERT_TRUE(Serial.outputContains("R:"));
    TEST_ASSERT_TRUE(Serial.outputContains("A:"));
    TEST_ASSERT_TRUE(Serial.outputContains("F:"));
    // No move commands should be produced
    TEST_ASSERT_TRUE(moveBuffer.isEmpty());
}

void test_m115_firmware_info() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    Serial.clearOutput();
    interpret(interp, gcodeBuffer, "M115");

    TEST_ASSERT_TRUE(Serial.outputContains("FIRMWARE:SandTable"));
    TEST_ASSERT_TRUE(Serial.outputContains("VERSION:1.0.0"));
    TEST_ASSERT_TRUE(moveBuffer.isEmpty());
}

void test_unknown_gcode_error() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    Serial.clearOutput();
    interpret(interp, gcodeBuffer, "G99");

    TEST_ASSERT_TRUE(Serial.outputContains("error"));
    TEST_ASSERT_TRUE(Serial.outputContains("G99"));
}

void test_unknown_mcode_error() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    Serial.clearOutput();
    interpret(interp, gcodeBuffer, "M99");

    TEST_ASSERT_TRUE(Serial.outputContains("error"));
    TEST_ASSERT_TRUE(Serial.outputContains("M99"));
}

void test_ok_response_on_valid_command() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    Serial.clearOutput();
    interpret(interp, gcodeBuffer, "G1 R10 A20");

    TEST_ASSERT_TRUE(Serial.outputContains("ok"));
}

void test_negative_coordinates() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 R-50 A-180");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -50.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -180.0f, cmd.theta);
}

void test_decimal_coordinates() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "G1 R12.5 A45.75");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.5f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.75f, cmd.theta);
}

void test_lowercase_accepted() {
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    interpret(interp, gcodeBuffer, "g1 r10 a20");

    MoveCommand cmd;
    moveBuffer.dequeue(cmd);
    TEST_ASSERT_EQUAL(MOVE_TO, cmd.type);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, cmd.radius);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, cmd.theta);
}

void test_state_machine_incremental() {
    // Verify the state machine processes one token per run() call
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;
    GCodeInterpreter interp(gcodeBuffer, moveBuffer, mockStatusQuery);

    GCodeLine gcode;
    strncpy(gcode.line, "G1 R10 A20", GCODE_LINE_MAX_LENGTH);
    gcodeBuffer.enqueue(gcode);

    // First run: IDLE → dequeue line, transition to PARSING
    bool worked = interp.run();
    TEST_ASSERT_TRUE(worked);
    TEST_ASSERT_TRUE(moveBuffer.isEmpty()); // Not executed yet

    // Subsequent runs: PARSING tokens one by one
    // "G1" "R10" "A20" then end-of-line → EXECUTE
    int parseRuns = 0;
    while (moveBuffer.isEmpty() && parseRuns < 20) {
        interp.run();
        parseRuns++;
    }

    // Should have produced the move command after several parse + execute steps
    TEST_ASSERT_TRUE(moveBuffer.size() > 0);
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_g1_absolute_move);
    RUN_TEST(test_g0_rapid_move);
    RUN_TEST(test_g1_radius_only_theta_nan);
    RUN_TEST(test_g1_theta_only_radius_nan);
    RUN_TEST(test_g91_relative_mode);
    RUN_TEST(test_g90_absolute_mode);
    RUN_TEST(test_g91_omitted_axis_zero);
    RUN_TEST(test_g28_home);
    RUN_TEST(test_g92_set_position);
    RUN_TEST(test_feedrate_emits_set_speed);
    RUN_TEST(test_feedrate_not_repeated);
    RUN_TEST(test_modal_motion_bare_coordinates);
    RUN_TEST(test_comment_stripping);
    RUN_TEST(test_multiple_gcodes_on_one_line);
    RUN_TEST(test_m0_stop);
    RUN_TEST(test_m2_stop);
    RUN_TEST(test_m114_status_report);
    RUN_TEST(test_m115_firmware_info);
    RUN_TEST(test_unknown_gcode_error);
    RUN_TEST(test_unknown_mcode_error);
    RUN_TEST(test_ok_response_on_valid_command);
    RUN_TEST(test_negative_coordinates);
    RUN_TEST(test_decimal_coordinates);
    RUN_TEST(test_lowercase_accepted);
    RUN_TEST(test_state_machine_incremental);

    return UNITY_END();
}
