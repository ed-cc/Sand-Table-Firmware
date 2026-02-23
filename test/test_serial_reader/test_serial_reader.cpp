// test_serial_reader.cpp - Unit tests for SerialReader
#include <unity.h>
#include "serial_reader.h"

// Mock globals
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// Test state for callbacks
static bool emergencyStopCalled = false;
static bool resumeCalled = false;
static bool statusQueryCalled = false;
static MachineStatus lastStatus;

void mockEmergencyStop() { emergencyStopCalled = true; }
void mockResume() { resumeCalled = true; }
void mockStatusQuery(MachineStatus& status) {
    statusQueryCalled = true;
    status.radius = 10.0f;
    status.theta = 45.0f;
    status.feedrate = 100.0f;
    status.isMoving = false;
}

static void resetTestState() {
    Serial.reset();
    emergencyStopCalled = false;
    resumeCalled = false;
    statusQueryCalled = false;
}

// Helper: feed all available chars through run()
static void feedAll(SerialReader& reader) {
    while (Serial.available()) {
        reader.run();
    }
}

// ===== Test Cases =====

void test_empty_serial_returns_false() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    TEST_ASSERT_FALSE(reader.run());
    TEST_ASSERT_TRUE(buf.isEmpty());
}

void test_single_line_enqueued() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    Serial.setInput("G1 R10 A20\n");
    feedAll(reader);

    TEST_ASSERT_EQUAL(1, buf.size());
    GCodeLine line;
    buf.dequeue(line);
    TEST_ASSERT_EQUAL_STRING("G1 R10 A20", line.line);
}

void test_empty_line_skipped() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    Serial.setInput("\n");
    feedAll(reader);

    TEST_ASSERT_TRUE(buf.isEmpty());
}

void test_carriage_return_ignored() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    Serial.setInput("G1 R10\r\n");
    feedAll(reader);

    TEST_ASSERT_EQUAL(1, buf.size());
    GCodeLine line;
    buf.dequeue(line);
    TEST_ASSERT_EQUAL_STRING("G1 R10", line.line);
}

void test_multiple_lines() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    Serial.setInput("G1 R10\nG1 A20\nG28\n");
    feedAll(reader);

    TEST_ASSERT_EQUAL(3, buf.size());

    GCodeLine line;
    buf.dequeue(line);
    TEST_ASSERT_EQUAL_STRING("G1 R10", line.line);
    buf.dequeue(line);
    TEST_ASSERT_EQUAL_STRING("G1 A20", line.line);
    buf.dequeue(line);
    TEST_ASSERT_EQUAL_STRING("G28", line.line);
}

void test_emergency_stop() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    Serial.setInput("!");
    feedAll(reader);

    TEST_ASSERT_TRUE(emergencyStopCalled);
    TEST_ASSERT_TRUE(buf.isEmpty());
}

void test_emergency_stop_discards_partial_line() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    // Start a line, then emergency stop, then a new line
    Serial.setInput("G1 R!G28\n");
    feedAll(reader);

    TEST_ASSERT_TRUE(emergencyStopCalled);
    // Only "G28" should be enqueued (partial "G1 R" was discarded by !)
    TEST_ASSERT_EQUAL(1, buf.size());
    GCodeLine line;
    buf.dequeue(line);
    TEST_ASSERT_EQUAL_STRING("G28", line.line);
}

void test_status_query() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    Serial.setInput("?");
    feedAll(reader);

    TEST_ASSERT_TRUE(statusQueryCalled);
    TEST_ASSERT_TRUE(buf.isEmpty());
    // Should have printed status response
    TEST_ASSERT_TRUE(Serial.outputContains("<Idle|R:"));
    TEST_ASSERT_TRUE(Serial.outputContains("10.00"));
    TEST_ASSERT_TRUE(Serial.outputContains("45.00"));
}

void test_resume() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    Serial.setInput("~");
    feedAll(reader);

    TEST_ASSERT_TRUE(resumeCalled);
    TEST_ASSERT_TRUE(buf.isEmpty());
}

void test_line_too_long_truncated() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    // Build a string longer than GCODE_LINE_MAX_LENGTH (96)
    char longInput[200];
    memset(longInput, 'A', 150);
    longInput[150] = '\n';
    longInput[151] = '\0';

    Serial.setInput(longInput);
    feedAll(reader);

    // Should still enqueue, just truncated
    TEST_ASSERT_EQUAL(1, buf.size());
    GCodeLine line;
    buf.dequeue(line);
    // Line should be exactly GCODE_LINE_MAX_LENGTH - 1 chars
    TEST_ASSERT_EQUAL(GCODE_LINE_MAX_LENGTH - 1, (int)strlen(line.line));
}

void test_realtime_chars_dont_interrupt_line() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    // Status query mid-line shouldn't affect the line being built
    // (? is consumed immediately, doesn't go into line buffer)
    Serial.setInput("G1?R10\n");
    feedAll(reader);

    TEST_ASSERT_TRUE(statusQueryCalled);
    TEST_ASSERT_EQUAL(1, buf.size());
    GCodeLine line;
    buf.dequeue(line);
    TEST_ASSERT_EQUAL_STRING("G1R10", line.line);
}

void test_buffer_full_rejects() {
    resetTestState();
    RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> buf;
    SerialReader reader(buf, mockEmergencyStop, mockStatusQuery, mockResume);

    // Fill the buffer
    for (int i = 0; i < GCODE_BUFFER_CAPACITY; i++) {
        GCodeLine line;
        snprintf(line.line, GCODE_LINE_MAX_LENGTH, "G%d", i);
        buf.enqueue(line);
    }
    TEST_ASSERT_TRUE(buf.isFull());

    // Try to add another line via serial
    Serial.setInput("G99\n");
    feedAll(reader);

    // Buffer should still be full with original items
    TEST_ASSERT_TRUE(buf.isFull());
    TEST_ASSERT_EQUAL(GCODE_BUFFER_CAPACITY, buf.size());
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_empty_serial_returns_false);
    RUN_TEST(test_single_line_enqueued);
    RUN_TEST(test_empty_line_skipped);
    RUN_TEST(test_carriage_return_ignored);
    RUN_TEST(test_multiple_lines);
    RUN_TEST(test_emergency_stop);
    RUN_TEST(test_emergency_stop_discards_partial_line);
    RUN_TEST(test_status_query);
    RUN_TEST(test_resume);
    RUN_TEST(test_line_too_long_truncated);
    RUN_TEST(test_realtime_chars_dont_interrupt_line);
    RUN_TEST(test_buffer_full_rejects);

    return UNITY_END();
}
