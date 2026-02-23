// Arduino.h - Mock for native unit testing
#ifndef Arduino_h
#define Arduino_h

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// Arduino constants
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

// F() macro - on AVR stores strings in flash; on native just passes through
#define F(x) (x)

// Controllable mock time (tests set this directly)
extern unsigned long mock_micros_value;

// Step counter (incremented on each HIGH write to a step pin)
extern long mock_step_count;

// Arduino-like functions
inline unsigned long micros() { return mock_micros_value; }
inline unsigned long millis() { return mock_micros_value / 1000; }
inline void digitalWrite(int pin, int value) {
    if (value == HIGH) mock_step_count++;
}
inline int digitalRead(int pin) { (void)pin; return HIGH; }
inline void pinMode(int pin, int mode) { (void)pin; (void)mode; }
inline void delayMicroseconds(unsigned int us) { (void)us; }
inline void delay(unsigned long ms) { (void)ms; }

// Arduino math macros
#ifndef abs
#define abs(x) ((x) < 0 ? -(x) : (x))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef constrain
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

// ===== Mock Serial =====
// Simulates Arduino Serial with controllable input and captured output

class MockSerial {
public:
    // Input buffer (set by tests via setInput)
    const char* _input;
    int _inputPos;
    int _inputLen;

    // Output capture
    char _output[4096];
    int _outputLen;

    MockSerial() : _input(0), _inputPos(0), _inputLen(0), _outputLen(0) {
        _output[0] = '\0';
    }

    void begin(unsigned long baud) { (void)baud; reset(); }

    void reset() {
        _input = 0;
        _inputPos = 0;
        _inputLen = 0;
        _outputLen = 0;
        _output[0] = '\0';
    }

    // Feed input characters for Serial.read()
    void setInput(const char* data) {
        _input = data;
        _inputPos = 0;
        _inputLen = (int)strlen(data);
    }

    int available() {
        return (_input && _inputPos < _inputLen) ? 1 : 0;
    }

    int read() {
        if (_input && _inputPos < _inputLen) return (unsigned char)_input[_inputPos++];
        return -1;
    }

    // Output helpers
    void _append(const char* s) {
        int len = (int)strlen(s);
        if (_outputLen + len < 4095) {
            memcpy(_output + _outputLen, s, len);
            _outputLen += len;
            _output[_outputLen] = '\0';
        }
    }

    // print overloads
    size_t print(const char* s)         { _append(s); return strlen(s); }
    size_t print(char c)                { char b[2] = {c, 0}; _append(b); return 1; }
    size_t print(int v)                 { char b[16]; snprintf(b, 16, "%d", v); _append(b); return strlen(b); }
    size_t print(long v)                { char b[16]; snprintf(b, 16, "%ld", v); _append(b); return strlen(b); }
    size_t print(unsigned long v)       { char b[16]; snprintf(b, 16, "%lu", v); _append(b); return strlen(b); }
    size_t print(unsigned int v, int base) { char b[16]; snprintf(b, 16, "%u", v); _append(b); return strlen(b); }
    size_t print(float v, int d)        { char b[32]; snprintf(b, 32, "%.*f", d, (double)v); _append(b); return strlen(b); }
    size_t print(double v, int d)       { char b[32]; snprintf(b, 32, "%.*f", d, v); _append(b); return strlen(b); }

    // println overloads
    size_t println()                    { _append("\n"); return 1; }
    size_t println(const char* s)       { print(s); _append("\n"); return strlen(s) + 1; }
    size_t println(int v)               { print(v); _append("\n"); return 1; }
    size_t println(long v)              { print(v); _append("\n"); return 1; }
    size_t println(float v, int d)      { print(v, d); _append("\n"); return 1; }
    size_t println(double v, int d)     { print(v, d); _append("\n"); return 1; }

    // For `while (!Serial)` compatibility
    explicit operator bool() const { return true; }

    // Test helpers
    bool outputContains(const char* needle) { return strstr(_output, needle) != 0; }
    void clearOutput() { _outputLen = 0; _output[0] = '\0'; }
};

extern MockSerial Serial;

#endif // Arduino_h
