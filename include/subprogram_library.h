// subprogram_library.h - PROGMEM-stored G-code subprograms
//
// Each subprogram is an array of G-code strings stored entirely in flash.
// The registry maps P-numbers (used by M98) to their line arrays.
//
#ifndef SUBPROGRAM_LIBRARY_H
#define SUBPROGRAM_LIBRARY_H

#include <avr/pgmspace.h>
#include <stdint.h>

// Descriptor for a stored subprogram (lives in PROGMEM)
struct Subprogram {
    uint8_t id;                    // P number for M98
    const char* const* lines;     // PROGMEM array of PGM_P string pointers
    uint8_t lineCount;            // number of lines in the program
};

// ============================================================================
// P1: Archimedean Spiral — center to edge
//
// Formula for regeneration:
//   ROTATIONS = 8, POINTS = 80, RADIUS = RADIUS_MAX_MM (290)
//   For i in 0..POINTS:
//     r = RADIUS * i / POINTS
//     theta = ROTATIONS * 360 * i / POINTS
//   Feedrate: 1200 mm/min
//
// To change density: adjust ROTATIONS. To change smoothness: adjust POINTS.
// ============================================================================

// -- Spiral G-code lines stored in flash --
static const char p1_L00[] PROGMEM = "G90";
static const char p1_L01[] PROGMEM = "G1 R0 A0 F1200";
// Point 1: r=3.63, theta=36.00
static const char p1_L02[] PROGMEM = "G1 R3.63 A36.00";
// Point 2: r=7.25, theta=72.00
static const char p1_L03[] PROGMEM = "G1 R7.25 A72.00";
// Point 3: r=10.88, theta=108.00
static const char p1_L04[] PROGMEM = "G1 R10.88 A108.00";
// Point 4: r=14.50, theta=144.00
static const char p1_L05[] PROGMEM = "G1 R14.50 A144.00";
// Point 5: r=18.13, theta=180.00
static const char p1_L06[] PROGMEM = "G1 R18.13 A180.00";
// Point 6: r=21.75, theta=216.00
static const char p1_L07[] PROGMEM = "G1 R21.75 A216.00";
// Point 7: r=25.38, theta=252.00
static const char p1_L08[] PROGMEM = "G1 R25.38 A252.00";
// Point 8: r=29.00, theta=288.00
static const char p1_L09[] PROGMEM = "G1 R29.00 A288.00";
// Point 9: r=32.63, theta=324.00
static const char p1_L10[] PROGMEM = "G1 R32.63 A324.00";
// Point 10: r=36.25, theta=360.00
static const char p1_L11[] PROGMEM = "G1 R36.25 A360.00";
// Point 11: r=39.88, theta=396.00
static const char p1_L12[] PROGMEM = "G1 R39.88 A396.00";
// Point 12: r=43.50, theta=432.00
static const char p1_L13[] PROGMEM = "G1 R43.50 A432.00";
// Point 13: r=47.13, theta=468.00
static const char p1_L14[] PROGMEM = "G1 R47.13 A468.00";
// Point 14: r=50.75, theta=504.00
static const char p1_L15[] PROGMEM = "G1 R50.75 A504.00";
// Point 15: r=54.38, theta=540.00
static const char p1_L16[] PROGMEM = "G1 R54.38 A540.00";
// Point 16: r=58.00, theta=576.00
static const char p1_L17[] PROGMEM = "G1 R58.00 A576.00";
// Point 17: r=61.63, theta=612.00
static const char p1_L18[] PROGMEM = "G1 R61.63 A612.00";
// Point 18: r=65.25, theta=648.00
static const char p1_L19[] PROGMEM = "G1 R65.25 A648.00";
// Point 19: r=68.88, theta=684.00
static const char p1_L20[] PROGMEM = "G1 R68.88 A684.00";
// Point 20: r=72.50, theta=720.00
static const char p1_L21[] PROGMEM = "G1 R72.50 A720.00";
// Point 21: r=76.13, theta=756.00
static const char p1_L22[] PROGMEM = "G1 R76.13 A756.00";
// Point 22: r=79.75, theta=792.00
static const char p1_L23[] PROGMEM = "G1 R79.75 A792.00";
// Point 23: r=83.38, theta=828.00
static const char p1_L24[] PROGMEM = "G1 R83.38 A828.00";
// Point 24: r=87.00, theta=864.00
static const char p1_L25[] PROGMEM = "G1 R87.00 A864.00";
// Point 25: r=90.63, theta=900.00
static const char p1_L26[] PROGMEM = "G1 R90.63 A900.00";
// Point 26: r=94.25, theta=936.00
static const char p1_L27[] PROGMEM = "G1 R94.25 A936.00";
// Point 27: r=97.88, theta=972.00
static const char p1_L28[] PROGMEM = "G1 R97.88 A972.00";
// Point 28: r=101.50, theta=1008.00
static const char p1_L29[] PROGMEM = "G1 R101.50 A1008.00";
// Point 29: r=105.13, theta=1044.00
static const char p1_L30[] PROGMEM = "G1 R105.13 A1044.00";
// Point 30: r=108.75, theta=1080.00
static const char p1_L31[] PROGMEM = "G1 R108.75 A1080.00";
// Point 31: r=112.38, theta=1116.00
static const char p1_L32[] PROGMEM = "G1 R112.38 A1116.00";
// Point 32: r=116.00, theta=1152.00
static const char p1_L33[] PROGMEM = "G1 R116.00 A1152.00";
// Point 33: r=119.63, theta=1188.00
static const char p1_L34[] PROGMEM = "G1 R119.63 A1188.00";
// Point 34: r=123.25, theta=1224.00
static const char p1_L35[] PROGMEM = "G1 R123.25 A1224.00";
// Point 35: r=126.88, theta=1260.00
static const char p1_L36[] PROGMEM = "G1 R126.88 A1260.00";
// Point 36: r=130.50, theta=1296.00
static const char p1_L37[] PROGMEM = "G1 R130.50 A1296.00";
// Point 37: r=134.13, theta=1332.00
static const char p1_L38[] PROGMEM = "G1 R134.13 A1332.00";
// Point 38: r=137.75, theta=1368.00
static const char p1_L39[] PROGMEM = "G1 R137.75 A1368.00";
// Point 39: r=141.38, theta=1404.00
static const char p1_L40[] PROGMEM = "G1 R141.38 A1404.00";
// Point 40: r=145.00, theta=1440.00
static const char p1_L41[] PROGMEM = "G1 R145.00 A1440.00";
// Point 41: r=148.63, theta=1476.00
static const char p1_L42[] PROGMEM = "G1 R148.63 A1476.00";
// Point 42: r=152.25, theta=1512.00
static const char p1_L43[] PROGMEM = "G1 R152.25 A1512.00";
// Point 43: r=155.88, theta=1548.00
static const char p1_L44[] PROGMEM = "G1 R155.88 A1548.00";
// Point 44: r=159.50, theta=1584.00
static const char p1_L45[] PROGMEM = "G1 R159.50 A1584.00";
// Point 45: r=163.13, theta=1620.00
static const char p1_L46[] PROGMEM = "G1 R163.13 A1620.00";
// Point 46: r=166.75, theta=1656.00
static const char p1_L47[] PROGMEM = "G1 R166.75 A1656.00";
// Point 47: r=170.38, theta=1692.00
static const char p1_L48[] PROGMEM = "G1 R170.38 A1692.00";
// Point 48: r=174.00, theta=1728.00
static const char p1_L49[] PROGMEM = "G1 R174.00 A1728.00";
// Point 49: r=177.63, theta=1764.00
static const char p1_L50[] PROGMEM = "G1 R177.63 A1764.00";
// Point 50: r=181.25, theta=1800.00
static const char p1_L51[] PROGMEM = "G1 R181.25 A1800.00";
// Point 51: r=184.88, theta=1836.00
static const char p1_L52[] PROGMEM = "G1 R184.88 A1836.00";
// Point 52: r=188.50, theta=1872.00
static const char p1_L53[] PROGMEM = "G1 R188.50 A1872.00";
// Point 53: r=192.13, theta=1908.00
static const char p1_L54[] PROGMEM = "G1 R192.13 A1908.00";
// Point 54: r=195.75, theta=1944.00
static const char p1_L55[] PROGMEM = "G1 R195.75 A1944.00";
// Point 55: r=199.38, theta=1980.00
static const char p1_L56[] PROGMEM = "G1 R199.38 A1980.00";
// Point 56: r=203.00, theta=2016.00
static const char p1_L57[] PROGMEM = "G1 R203.00 A2016.00";
// Point 57: r=206.63, theta=2052.00
static const char p1_L58[] PROGMEM = "G1 R206.63 A2052.00";
// Point 58: r=210.25, theta=2088.00
static const char p1_L59[] PROGMEM = "G1 R210.25 A2088.00";
// Point 59: r=213.88, theta=2124.00
static const char p1_L60[] PROGMEM = "G1 R213.88 A2124.00";
// Point 60: r=217.50, theta=2160.00
static const char p1_L61[] PROGMEM = "G1 R217.50 A2160.00";
// Point 61: r=221.13, theta=2196.00
static const char p1_L62[] PROGMEM = "G1 R221.13 A2196.00";
// Point 62: r=224.75, theta=2232.00
static const char p1_L63[] PROGMEM = "G1 R224.75 A2232.00";
// Point 63: r=228.38, theta=2268.00
static const char p1_L64[] PROGMEM = "G1 R228.38 A2268.00";
// Point 64: r=232.00, theta=2304.00
static const char p1_L65[] PROGMEM = "G1 R232.00 A2304.00";
// Point 65: r=235.63, theta=2340.00
static const char p1_L66[] PROGMEM = "G1 R235.63 A2340.00";
// Point 66: r=239.25, theta=2376.00
static const char p1_L67[] PROGMEM = "G1 R239.25 A2376.00";
// Point 67: r=242.88, theta=2412.00
static const char p1_L68[] PROGMEM = "G1 R242.88 A2412.00";
// Point 68: r=246.50, theta=2448.00
static const char p1_L69[] PROGMEM = "G1 R246.50 A2448.00";
// Point 69: r=250.13, theta=2484.00
static const char p1_L70[] PROGMEM = "G1 R250.13 A2484.00";
// Point 70: r=253.75, theta=2520.00
static const char p1_L71[] PROGMEM = "G1 R253.75 A2520.00";
// Point 71: r=257.38, theta=2556.00
static const char p1_L72[] PROGMEM = "G1 R257.38 A2556.00";
// Point 72: r=261.00, theta=2592.00
static const char p1_L73[] PROGMEM = "G1 R261.00 A2592.00";
// Point 73: r=264.63, theta=2628.00
static const char p1_L74[] PROGMEM = "G1 R264.63 A2628.00";
// Point 74: r=268.25, theta=2664.00
static const char p1_L75[] PROGMEM = "G1 R268.25 A2664.00";
// Point 75: r=271.88, theta=2700.00
static const char p1_L76[] PROGMEM = "G1 R271.88 A2700.00";
// Point 76: r=275.50, theta=2736.00
static const char p1_L77[] PROGMEM = "G1 R275.50 A2736.00";
// Point 77: r=279.13, theta=2772.00
static const char p1_L78[] PROGMEM = "G1 R279.13 A2772.00";
// Point 78: r=282.75, theta=2808.00
static const char p1_L79[] PROGMEM = "G1 R282.75 A2808.00";
// Point 79: r=286.38, theta=2844.00
static const char p1_L80[] PROGMEM = "G1 R286.38 A2844.00";
// Point 80: r=290.00, theta=2880.00
static const char p1_L81[] PROGMEM = "G1 R290.00 A2880.00";

// PROGMEM array of pointers to the G-code strings
static const char* const p1_lines[] PROGMEM = {
    p1_L00, p1_L01, p1_L02, p1_L03, p1_L04, p1_L05, p1_L06, p1_L07,
    p1_L08, p1_L09, p1_L10, p1_L11, p1_L12, p1_L13, p1_L14, p1_L15,
    p1_L16, p1_L17, p1_L18, p1_L19, p1_L20, p1_L21, p1_L22, p1_L23,
    p1_L24, p1_L25, p1_L26, p1_L27, p1_L28, p1_L29, p1_L30, p1_L31,
    p1_L32, p1_L33, p1_L34, p1_L35, p1_L36, p1_L37, p1_L38, p1_L39,
    p1_L40, p1_L41, p1_L42, p1_L43, p1_L44, p1_L45, p1_L46, p1_L47,
    p1_L48, p1_L49, p1_L50, p1_L51, p1_L52, p1_L53, p1_L54, p1_L55,
    p1_L56, p1_L57, p1_L58, p1_L59, p1_L60, p1_L61, p1_L62, p1_L63,
    p1_L64, p1_L65, p1_L66, p1_L67, p1_L68, p1_L69, p1_L70, p1_L71,
    p1_L72, p1_L73, p1_L74, p1_L75, p1_L76, p1_L77, p1_L78, p1_L79,
    p1_L80, p1_L81
};

#define P1_LINE_COUNT 82

// ============================================================================
// Subprogram Registry
// ============================================================================

static const Subprogram subprogramRegistry[] PROGMEM = {
    { 1, p1_lines, P1_LINE_COUNT },
};

#define SUBPROGRAM_COUNT 1

// Look up a subprogram by its P number.
// Returns pointer into PROGMEM registry, or nullptr if not found.
// Caller must use pgm_read_* to access struct fields.
inline const Subprogram* findSubprogram(uint8_t id) {
    for (uint8_t i = 0; i < SUBPROGRAM_COUNT; i++) {
        uint8_t progId = pgm_read_byte(&subprogramRegistry[i].id);
        if (progId == id) {
            return &subprogramRegistry[i];
        }
    }
    return nullptr;
}

#endif // SUBPROGRAM_LIBRARY_H
