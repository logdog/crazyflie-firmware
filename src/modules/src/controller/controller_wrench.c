

#include "controller_wrench.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "debug.h"


// static float cmd_thrust;
// static float cmd_roll;
// static float cmd_pitch;

void controllerWrenchInit(void)
{
}

bool controllerWrenchTest(void)
{
  return true;
}

void controllerWrench(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  float commandedThrustPercent = setpoint->thrust / 65535.0f;

  control->controlMode = controlModeWrench;
  control->Fx = 0.0f; // N
  control->Fy = 0.0f; // N
  control->Fz = commandedThrustPercent * 650.0f * 0.009866f; // convert to N using grams as intermediate step
  control->Tx = 0.0f; // Nm
  control->Ty = 0.0f; // Nm
  control->Tz = 0.0f; // Nm
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
// LOG_GROUP_START(controller)
// LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
// LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
// LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
// LOG_GROUP_STOP(controller)
