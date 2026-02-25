#include <Arduino.h>
#include "motion.h"
#include "serial_reader.h"
#include "gcode_interpreter.h"
#include "motion_commander.h"
#include "subprogram_runner.h"

// Motion controller
Motion motion;

// Buffers
RingBuffer<GCodeLine, GCODE_BUFFER_CAPACITY> gcodeBuffer;
RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY> moveBuffer;

// Subprogram runner (feeds PROGMEM G-code into the pipeline)
SubprogramRunner subRunner(gcodeBuffer);

// Callback for M98 subprogram calls
bool startSubprogram(uint8_t programNumber, uint16_t repetitions)
{
  if (subRunner.isActive()) {
    Serial.println(F("error: Subprogram already running"));
    return false;
  }
  return subRunner.start(programNumber, repetitions);
}

// Callbacks
void emergencyStop()
{
  subRunner.abort();
  motion.disableMotors();
}

void getStatus(MachineStatus &status)
{
  status.radius = motion.getRadiusMM();
  status.theta = motion.getThetaDegrees();
  status.feedrate = motion.getFeedrate();
  status.isMoving = !motion.isMovementComplete();
}

void onResume()
{
  motion.enableMotors();
}

// Pipeline stages
SerialReader serialReader(gcodeBuffer, emergencyStop, getStatus, onResume);
GCodeInterpreter gcodeInterpreter(gcodeBuffer, moveBuffer, getStatus);
MotionCommander motionCommander(moveBuffer, motion);

// Round-robin counter for pipeline scheduling
uint8_t pipelineSlot = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    delay(1);
  Serial.println(F("Sand Table - Motion Control System"));
  Serial.println(F("==================================="));

  motion.setupSteppers();

  gcodeInterpreter.setSubprogramCallback(startSubprogram);

  Serial.println(F("Ready. Awaiting G-code commands."));
}

void loop()
{
  // Stepper timing is the highest priority
  motion.run();

  // Round-robin one pipeline stage per iteration
  switch (pipelineSlot)
  {
  case 0:
    subRunner.run();    // feed PROGMEM lines when active (no-op otherwise)
    serialReader.run(); // always runs for real-time commands (!?~)
    break;
  case 1:
    gcodeInterpreter.run();
    break;
  case 2:
    motionCommander.run();
    break;
  }
  pipelineSlot = (pipelineSlot + 1) % 3;
}
