#include <Arduino.h>
#include "motion.h"

// Create motion controller instance
Motion motion;

bool homed = false;

void setup()
{
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ; // Wait up to 3 seconds for Serial
  Serial.println(F("Sand Table - Motion Control System"));
  Serial.println(F("==================================="));

  // Initialize motion system (TMC drivers and steppers)
  motion.setupSteppers();

  Serial.println(F("Setup complete! Ready for motion.\n"));
}

void loop()
{
  // Run the steppers (non-blocking)
  motion.run();

  if (!homed)
  {
    // Start homing sequence
    motion.home();
    homed = true;
  }
  while( motion.isHoming() )
  {
    motion.run();
  }

  // Example: Simple test movement every 5 seconds
  static unsigned long lastMove = 0;
  static bool moveState = false;

  if (millis() - lastMove > 5000)
  {
    lastMove = millis();
    moveState = !moveState;

    if (moveState)
    {
      motion.moveTo(100.0, 90.0);
    }
    else
    {
      motion.moveTo(100.0, 360.0);
    }
  }
}
