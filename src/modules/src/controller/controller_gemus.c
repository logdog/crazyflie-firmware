

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
};

static struct schenato_e baseSchenatoProfile = {
    .A = 1.0f,
    .K = 29.0f,
    .rho = 0.33,
    .gamma = 10.0f
};

static struct schenato_e leftSchenatoProfile = {
    .A = 1.0f,
    .K = 29.0f,
    .rho = 0.33,
    .gamma = 10.0f
};

static struct schenato_e rightSchenatoProfile = {
    .A = 1.0f,
    .K = 29.0f,
    .rho = 0.33,
    .gamma = 10.0f
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
// float calculateNormalizedTime(float time_s, struct schenato_e s) {
//     return s.T * (time_s / (s.T) - (float) floor(time_s / (s.T)));
// }

// t is the normalized time [0,1)
// see Taha, 2012 Review of Flapping Wing Vehicle with T = 1
float calculateSchenatoAngle(float t, struct schenato_e s) {
    
    float angle;

    if (0 <= t && t <= s.rho) {
        angle = s.A * (1 + s.K) * (1 - 2.0f*t/s.rho) + s.gamma * s.A;
    }
    else {
        angle = s.A * (1 + s.K) * (2.0f*(t - s.rho)/(1 - s.rho) - 1) + s.gamma * s.A;
    }

    return angle;
}

struct controller_gemus_log_e {
  float servo1_deg;
  float servo2_deg;
  float servo3_deg;
  float servo4_deg;
};

static struct controller_gemus_log_e gemus_log;

static float t = 0.0f; // normalized time
static float T = 1.0f; // flapping period

void controllerGemus(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlMode4Servos;

  // setpoint->thrust
  // thrust = 0 --> minimum flapping Hz
  // thrust = 60000 --> maximum flapping Hz
  float frequency = 1.0f + 1.0f * (setpoint->thrust / 60000.0f);

  // put some safety bounds on the frequency
  if (frequency > 2.0f) {
    frequency = 2.0f;
  }
  else if (frequency < 1.0f) {
    frequency = 1.0f;
  }

  T = 1.0f/frequency;

  t += 1 / (T * 1000.0f);
  while (t >= 1.0f) {
    t -= 1.0f;
  }

  leftSchenatoProfile = baseSchenatoProfile;
  rightSchenatoProfile = baseSchenatoProfile;

  // differential flapping angle: use roll command to make turns
  // roll command is -30 to -30 --> need to map to multiplier of 0.9 to 1.1
  leftSchenatoProfile.A = baseSchenatoProfile.A * (1.0f + 0.2f*setpoint->attitude.roll/30.0f);
  rightSchenatoProfile.A = baseSchenatoProfile.A * (1.0f - 0.2f*setpoint->attitude.roll/30.0f);

  control->servo1_deg = calculateSchenatoAngle(t, leftSchenatoProfile); // t is normalized time
  control->servo2_deg = calculateSchenatoAngle(t, rightSchenatoProfile);

  // just for testing, so the same thing to servo3_deg and servo4_deg
  control->servo3_deg = setpoint->attitude.pitch;
  control->servo4_deg = setpoint->attitude.pitch;

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

PARAM_ADD(PARAM_FLOAT, A,       &baseSchenatoProfile.A)
PARAM_ADD(PARAM_FLOAT, K,       &baseSchenatoProfile.K)
PARAM_ADD(PARAM_FLOAT, rho,     &baseSchenatoProfile.rho)
PARAM_ADD(PARAM_FLOAT, gamma,   &baseSchenatoProfile.gamma)
PARAM_ADD(PARAM_FLOAT, T,       &T)

PARAM_GROUP_STOP(gemus)