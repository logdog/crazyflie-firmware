

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
  float normalizedThrust = setpoint->thrust / 65000.0f; // max thrust is 65000, NOT 65535
  float thrust_g = normalizedThrust * 650.0f; // thrust in grams
  float thrust_N = thrust_g * 0.00980665f; // thrust in N

  control->controlMode = controlModeWrench;
  control->Fx = 0.0f; // N
  control->Fy = 0.0f; // N
  control->Fz = thrust_N; // N
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
