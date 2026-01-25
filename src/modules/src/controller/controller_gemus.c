

#include "controller_gemus.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "debug.h"


static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;

void controllerGemusInit(void)
{
}

bool controllerGemusTest(void)
{
  return true;
}

void controllerGemus(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlMode4Servos;
  control->thrust = setpoint->thrust / 65000.0f; // normalized 0 to 1
  control->roll = setpoint->attitude.roll;
  control->pitch = setpoint->attitude.pitch;

  cmd_thrust = control->thrust;
  cmd_roll = control->roll;
  cmd_pitch = control->pitch;

//   if (stabilizerStep % 1000 == 0) {
//     DEBUG_PRINT("setpoint->thrust: %f\n", (double) setpoint->thrust);
//   }
}


// LOG_GROUP_START(controller)

// LOG_GROUP_STOP(controller)
