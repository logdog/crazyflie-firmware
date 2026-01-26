

#include "controller_gemus.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "debug.h"


struct schenato_e {
  float A;
  float K;
  float rho;
  float gamma;
  float T;
};

static struct schenato_e leftSchenatoProfile = {
    .A = 1.0f,
    .K = 8.0f,
    .rho = 0.25f,
    .gamma = 0.1f,
    .T = 1.0f,
};

// static struct schenato_e rightSchenatoProfile = {
//     .A = 1.0f,
//     .K = 8.0f,
//     .rho = 0.25f,
//     .gamma = 0.1f,
//     .T = 1.0f,
// };

void controllerGemusInit(void)
{
}

bool controllerGemusTest(void)
{
  return true;
}

// time_s is the time in seconds
float calculateNormalizedTime(float time_s, struct schenato_e s) {
    return s.T * (time_s / (s.T) - (float) floor(time_s / (s.T)));
}

// t is the normalized time [0,1)
float calculateSchenatoAngle(float t, struct schenato_e s) {
    
    float angle;

    if (0 <= t && t <= s.rho * s.T) {
        angle = s.A * (1 + s.K) * (1 - 2.0f*t/(s.rho*s.T)) + s.gamma*s.A;
    }
    else {
        angle = s.A * (1 + s.K) * (2.0f*(t - s.rho * s.T)/((1 - s.rho)*s.T) - 1) + s.gamma * s.A;
    }

    return angle;
}

static bool enabled = true;

struct controller_gemus_log_e {
  float servo1_deg;
  float servo2_deg;
  float servo3_deg;
  float servo4_deg;
};

static struct controller_gemus_log_e gemus_log;

void controllerGemus(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlMode4Servos;

  // give user the option to disable controller via a parameter update.
  // This is a convenience feature for when testing on static stand.
  if (!enabled) {
    return;
  }

//   control->thrust = setpoint->thrust / 65000.0f; // normalized 0 to 1
//   control->roll = setpoint->attitude.roll;
//   control->pitch = setpoint->attitude.pitch;

  // time_s is the time in seconds
  // t is the normalized time [0,1) in the flapping period [0,T)
  float time_s = (float) stabilizerStep / 1000.0f;
  float t = calculateNormalizedTime(time_s, leftSchenatoProfile);

  control->servo1_deg = calculateSchenatoAngle(t, leftSchenatoProfile);
  control->servo2_deg = calculateSchenatoAngle(t, leftSchenatoProfile);

  // just for testing, so the same thing to servo3_deg and servo4_deg
  control->servo3_deg = control->servo1_deg;
  control->servo4_deg = control->servo1_deg;

  // log variables
  gemus_log.servo1_deg = control->servo1_deg;
  gemus_log.servo2_deg = control->servo2_deg;
  gemus_log.servo3_deg = control->servo3_deg;
  gemus_log.servo4_deg = control->servo4_deg;
}


LOG_GROUP_START(gemus)

LOG_ADD(LOG_FLOAT, servo1_deg, &gemus_log.servo1_deg)
LOG_ADD(LOG_FLOAT, servo2_deg, &gemus_log.servo2_deg)
LOG_ADD(LOG_FLOAT, servo3_deg, &gemus_log.servo3_deg)
LOG_ADD(LOG_FLOAT, servo4_deg, &gemus_log.servo4_deg)

LOG_GROUP_STOP(gemus)

PARAM_GROUP_START(gemus)

PARAM_ADD(PARAM_FLOAT, A,       &leftSchenatoProfile.A)
PARAM_ADD(PARAM_FLOAT, K,       &leftSchenatoProfile.K)
PARAM_ADD(PARAM_FLOAT, rho,     &leftSchenatoProfile.rho)
PARAM_ADD(PARAM_FLOAT, gamma,   &leftSchenatoProfile.gamma)
PARAM_ADD(PARAM_FLOAT, T,       &leftSchenatoProfile.T)

// only used for testing
PARAM_ADD(PARAM_1BYTE, enabled, &enabled)

PARAM_GROUP_STOP(gemus)