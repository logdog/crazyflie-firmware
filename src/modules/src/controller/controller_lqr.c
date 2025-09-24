#include <math.h>
#include <string.h>

#include "math3d.h"
#include "controller_lqr.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

#include "debug.h"
#include "config.h"

extern const unsigned int fh_lqr_max_index;
extern const float u0[];
extern const float k0[];
extern const float x0[];
extern const float K[];
float get_u0(unsigned int sample_index, unsigned int state_index) { return u0[4*sample_index + state_index]; }
float get_k0(unsigned int sample_index, unsigned int state_index) { return k0[4*sample_index + state_index]; }
float get_x0(unsigned int sample_index, unsigned int state_index) { return x0[12*sample_index + state_index]; }
float get_K(unsigned int sample_index, unsigned int row, unsigned int col) { return K[48*sample_index + 12*row + col]; }

#if defined(CONFIG_BATTERY_550)
static controllerLQR_t g_self = {
  // .k1 = {0.08737846f, 0.00518231f, 0.00000000f,
  //        -0.04022944f, 0.61655673f, -0.20388516f, 0.13644939f, 0.00831614f, -0.00000000f, -0.00721764f, 0.08684703f, -0.04707753f},

  // .k2 = {0.08737846f, -0.00518231f, 0.00000000f,
  //        0.04022944f, 0.61655673f, 0.20388516f, 0.13644939f, -0.00831614f, 0.00000000f, 0.00721764f, 0.08684703f, 0.04707753f},

  // .k3 = {-0.00000000f, -0.57855870f, 2.02812104f,
  //        3.63218699f, -0.00000000f, -0.08074192f, 0.00000000f, -0.87359421f, 2.26742355f, 0.39347115f, 0.00000000f, -0.01167539f},

  // .k4 = {0.00000000f, 0.57855870f, 2.02812104f,
  //        -3.63218699f, 0.00000000f, 0.08074192f, 0.00000000f, 0.87359421f, 2.26742355f, -0.39347115f, 0.00000000f, 0.01167539f},

  .k1 = {0.08737846f, -0.00518231f, 0.00000000f,
         0.04022944f, 0.61655673f, -0.20388516f, 0.13644939f, -0.00831614f, 0.00000000f, 0.00721764f, 0.08684703f, -0.04707753f},

  .k2 = {0.08737846f, 0.00518231f, 0.00000000f,
         -0.04022944f, 0.61655673f, 0.20388516f, 0.13644939f, 0.00831614f, -0.00000000f, -0.00721764f, 0.08684703f, 0.04707753f},

  .k3 = {0.00000000f, -0.57855870f, 2.02812104f,
         3.63218699f, 0.00000000f, 0.08074192f, 0.00000000f, -0.87359421f, 2.26742355f, 0.39347115f, 0.00000000f, 0.01167539f},

  .k4 = {-0.00000000f, 0.57855870f, 2.02812104f,
         -3.63218699f, -0.00000000f, -0.08074192f, 0.00000000f, 0.87359421f, 2.26742355f, -0.39347115f, 0.00000000f, -0.01167539f},

  .mass = 0.470f // kg
};
#else // CONFIG_BATTERY_1550
// updated controller that uses correct signs for all 4 inputs of the bicopter
// static controllerLQR_t g_self = {
//   .k1 = {0.08371652f, -0.00621467f, 0.00000000f,
//          0.04511223f, 0.53510613f, -0.20028632f, 0.12704892f, -0.00977868f, -0.00000000f, 0.00705775f, 0.05985790f, -0.03927555f},

//   .k2 = {0.08371652f, 0.00621467f, 0.00000000f,
//          -0.04511223f, 0.53510613f, 0.20028632f, 0.12704892f, 0.00977868f, 0.00000000f, -0.00705775f, 0.05985790f, 0.03927555f},

//   .k3 = {0.00000000f, -0.57378878f, 2.05772639f,
//          3.56149929f, 0.00000000f, 0.09668341f, 0.00000000f, -0.86363577f, 2.34124778f, 0.37319333f, 0.00000000f, 0.01287597f},

//   .k4 = {0.00000000f, 0.57378878f, 2.05772639f,
//          -3.56149929f, 0.00000000f, -0.09668341f, -0.00000000f, 0.86363577f, 2.34124778f, -0.37319333f, -0.00000000f, -0.01287597f},

//   .mass = 0.575f // kg
// };
static controllerLQR_t g_self = {
  .k1 = {0.09149180f, -0.00926999f, 0.00000000f,
         0.04898391f, 0.57753220f, -0.21136620f, 0.13835939f, -0.01162586f, 0.00000000f, 0.00733346f, 0.06230944f, -0.04034881f},

  .k2 = {0.09149180f, 0.00926999f, -0.00000000f,
         -0.04898391f, 0.57753220f, 0.21136620f, 0.13835939f, 0.01162586f, 0.00000000f, -0.00733346f, 0.06230944f, 0.04034881f},

  .k3 = {0.00000000f, -0.89836564f, 2.14498959f,
         4.00516745f, 0.00000000f, 0.10302765f, 0.00000000f, -1.06635256f, 2.42921273f, 0.39774998f, 0.00000000f, 0.01305347f},

  .k4 = {0.00000000f, 0.89836564f, 2.14498959f,
         -4.00516745f, 0.00000000f, -0.10302765f, 0.00000000f, 1.06635256f, 2.42921273f, -0.39774998f, 0.00000000f, -0.01305347f},

  .mass = 0.575f
  };

// static controllerLQR_t g_self = {
//   .k1 = {0.08371652f, -0.00859505f, -0.00000000f,
//          0.04613514f, 0.53510613f, -0.20030469f, 0.12704892f, -0.01083762f, -0.00000000f, 0.00710762f, 0.05985790f, -0.03927773f},

//   .k2 = {0.08371652f, 0.00859505f, 0.00000000f,
//          -0.04613514f, 0.53510613f, 0.20030469f, 0.12704892f, 0.01083762f, 0.00000000f, -0.00710762f, 0.05985790f, 0.03927773f},

//   .k3 = {-0.00000000f, -0.80922740f, 2.05772639f,
//          3.65878927f, -0.00000000f, 0.09496807f, 0.00000000f, -0.96491855f, 2.34124778f, 0.37790522f, -0.00000000f, 0.01267421f},

//   .k4 = {0.00000000f, 0.80922740f, 2.05772639f,
//          -3.65878927f, 0.00000000f, -0.09496807f, 0.00000000f, 0.96491855f, 2.34124778f, -0.37790522f, 0.00000000f, -0.01267421f},

//   .mass = 0.575f,
// };

#endif

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void controllerLQRReset(controllerLQR_t* self)
{
}

void controllerLQRInit(controllerLQR_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  controllerLQRReset(self);
}

bool controllerLQRTest(controllerLQR_t* self)
{
  return true;
}

// parameters (can be set over the air via the crazyflie radio)
enum LQR_MODES {
  DEFAULT = 0,          // infinite horizon, default controller
  FINITE_HORIZON = 1,   // finite horizon controller (not working)
  MANUAL_ROLL = 2,      // set absolute roll angle
  MANUAL_PITCH = 4,     // set absolute pitch angle
  MANUAL_Z_RATE = 8,    // set absolute (inertial frame) z velocity
};

static uint8_t lqr_mode = DEFAULT;

// flapping parameters
struct flappingConfig_s {
    enum flappingMode_s {
      disabled = 0,
      waitToEnable = 1,
      rampingUp = 2,
      enabled = 3,
      waitToDisable = 4,
    } state;
    float hz;
    float amplitudeDeg;
    uint32_t lastTick;
};

struct flappingConfig_s flappingConfig = {
  .state = disabled,
  .hz = 10,
  .amplitudeDeg = 10,
  .lastTick = 0,
};

// set the previous last value
static float lastServoLeftDeg = 0.0f;
static float lastServoRightDeg = 0.0f;

// logging variables
static float px, py, pz;
static float vx, vy, vz;
static float roll, pitch, yaw;
static float wx, wy, wz;
static float leftMotor;
static float rightMotor;
static float leftServo;
static float rightServo;
static float flappingOffset;

static uint8_t pitchMode;
static uint8_t rollMode;
static uint8_t zMode;
static float sXPos, sYPos, sZPos;
static float sXVel, sYVel, sZVel;
static float sRoll, sPitch, sYaw;

// averaging filter on the angular velocities
#define FILTER_LENGTH 5
static float filter_wx[FILTER_LENGTH] = {0.0f};
static float filter_wy[FILTER_LENGTH] = {0.0f};
static float filter_wz[FILTER_LENGTH] = {0.0f};
static int filter_count = 0;

// counter variables
static unsigned int lqr_count = 0;
static unsigned int fh_lqr_count = 0;
void controllerLQR(controllerLQR_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  // updates at 1 kHz

  // fill the circular buffer
  filter_wx[filter_count] = sensors->gyro.x;
  filter_wy[filter_count] = sensors->gyro.y;
  filter_wz[filter_count] = sensors->gyro.z;
  filter_count++;
  filter_count %= FILTER_LENGTH;

  float wx_avg = 0.0f;
  float wy_avg = 0.0f;
  float wz_avg = 0.0f;

  for (int i = 0; i < FILTER_LENGTH; i++) {
    wx_avg += filter_wx[i];
    wy_avg += filter_wy[i];
    wz_avg += filter_wz[i];
  }
  wx_avg /= (float) FILTER_LENGTH;
  wy_avg /= (float) FILTER_LENGTH;
  wz_avg /= (float) FILTER_LENGTH;

  // add the flapping control signal
  float flappingAngleOffsetDeg = 0.0f;
  
  // State Machine Transition Rules:
  // 1. The flappingConfig.state will be set to "waitToEnable", "waitToDisable" (or "disable")
  //    via a parameter over the air
  // 2. Only transition to start/stop flapping at whole second increments
  // 3. The rampingUp state will last 1 second
  if (flappingConfig.state == waitToEnable && (tick % 1000) == 0) {
    flappingConfig.state = rampingUp;
    flappingConfig.lastTick = tick;
  }
  else if (flappingConfig.state == rampingUp && tick - flappingConfig.lastTick >= 1000) {
    flappingConfig.state = enabled;
  }
  else if (flappingConfig.state == waitToDisable && (tick % 1000) == 0) {
    flappingConfig.state = disabled;
  }

  // calculate the flapping offset angle
  if (flappingConfig.state == rampingUp || flappingConfig.state == enabled || flappingConfig.state == waitToDisable) {
    float multiplier = 1.0f;
    if (flappingConfig.state == rampingUp) {
      multiplier = (tick - flappingConfig.lastTick) / 1000.0f;
    }
    flappingAngleOffsetDeg = multiplier * flappingConfig.amplitudeDeg * sinf(2*(float)M_PI*flappingConfig.hz*tick/1000.0f);
  }

  // always update the servo commands at 1kHz
  if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    control->servoLeft_deg = flappingAngleOffsetDeg + lastServoLeftDeg;   // when not flapping: 0 deg
    control->servoRight_deg = flappingAngleOffsetDeg + lastServoRightDeg; // when not flapping: 0 deg
    return;
  }

  lqr_count++;
  control->controlMode = controlModeLQR;

  // logging
  rollMode = setpoint->mode.roll;
  pitchMode = setpoint->mode.pitch;
  zMode = setpoint->mode.z;
  sRoll = setpoint->attitude.roll;
  sPitch = setpoint->attitude.pitch;
  sZVel = setpoint->velocity.z;

  sXPos = setpoint->position.x;
  sYPos = setpoint->position.y;
  sZPos = setpoint->position.z;
  sZVel = setpoint->velocity.z;

  float x[12] = {state->position.x, state->position.y, state->position.z,
                 radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw),
                 state->velocity.x, state->velocity.y, state->velocity.z,
                 radians(wx_avg), radians(wy_avg), radians(wz_avg)};
  // x[9] = 0;
  // x[10] = 0;
  // x[11] = 0;

  float xd[12] = {0};

  // althold flight mode. Values set with commander.send_setpoint()
  if (setpoint->mode.roll == modeAbs && setpoint->mode.pitch == modeAbs && setpoint->mode.z == modeVelocity) {
    xd[0] = x[0];  // x
    xd[1] = x[1];  // y
    xd[2] = x[2];  // z
    xd[3] = radians(setpoint->attitude.roll);
    xd[4] = -radians(setpoint->attitude.pitch);
    xd[5] = 0.0f;  // yaw
    xd[6] = x[6];  // x velocity
    xd[7] = x[7];  // y velocity
    xd[8] = setpoint->velocity.z;
  }
  // original flight mode
  else {
    // original flight mode
    xd[0] = setpoint->position.x;
    xd[1] = setpoint->position.y;
    xd[2] = setpoint->position.z;
  }

  // float maxPitch = 20.0f;
  // if ((setpoint->mode.pitch == modeAbs) && (-maxPitch < setpoint->attitude.pitch) && (setpoint->attitude.pitch < maxPitch)) {
  //   // eliminate control action due to x position and x velocity errors
  //   xd[0] = x[0];
  //   xd[6] = x[6];
  //   xd[4] = -radians(setpoint->attitude.pitch);
  // }

  // float maxRoll = 20.0f;
  // if ((lqr_mode & MANUAL_ROLL) && (-maxRoll < setpoint->attitude.roll && setpoint->attitude.roll < maxRoll)) {
  //   // eliminate control action due to y position and y velocity errors
  //   xd[1] = x[1];
  //   xd[7] = x[7];
  //   xd[3] = radians(setpoint->attitude.roll);
  // }

  // // set absolute (inertial frame) z velocity
  // float maxZRate = 2.0f;
  // if ((lqr_mode & MANUAL_Z_RATE) && (-maxZRate < setpoint->velocity.z && setpoint->velocity.z < maxZRate)) {
  //   xd[2] = x[2];
  //   xd[8] = setpoint->velocity.z;
  // }

  // for better landing
  // if (setpoint->mode.z == modeDisable) {
  //   control->motorLeft_N = 0.0f;
  //   control->motorRight_N = 0.0f;
  //   control->servoLeft_deg = 0.0f;
  //   control->servoRight_deg = 0.0f;
  //   return;
  // }

  if (lqr_mode == FINITE_HORIZON && fh_lqr_count < fh_lqr_max_index) {
  // if (false) {
    // FH LQR Controller
    // u = -K(t) (x - x0(t)) - k0(t) + u0(t)
    float tmp[4] = { 0.0f };

    for (int row = 0; row < 4; row++) {
      for (int i = 0; i < 12; i++) {
        tmp[row] += -get_K(fh_lqr_count, row, i) * (x[i] - get_x0(fh_lqr_count, i));
      }
      tmp[row] += -get_k0(fh_lqr_count, row);
      tmp[row] += get_u0(fh_lqr_count, row);
    }

    control->servoLeft_deg = degrees(tmp[0]);
    control->servoRight_deg = degrees(tmp[1]);
    control->motorLeft_N = tmp[2];
    control->motorRight_N = tmp[3];

    // update the counter to step through the trajectory
    fh_lqr_count++;
    
    // return to the standard infinite horizon LQR controller
    if (fh_lqr_count >= fh_lqr_max_index) {
      lqr_mode = DEFAULT;
      fh_lqr_count = 0;
    }
  }
  else {
    // Original LQR Controller
    // u = -K(x - x_desired) + u0
    float tmp = 0;
    for (int i = 0; i < 12; i++) {
      tmp += -self->k1[i] * (x[i] - xd[i]);
    }
    control->servoLeft_deg = degrees(tmp) + flappingAngleOffsetDeg;
    lastServoLeftDeg = degrees(tmp);
    
    tmp = 0;
    for (int i = 0; i < 12; i++) {
      tmp += -self->k2[i] * (x[i] - xd[i]);
    }
    control->servoRight_deg = degrees(tmp) + flappingAngleOffsetDeg;
    lastServoRightDeg = degrees(tmp);

    tmp = 0;
    for (int i = 0; i < 12; i++) {
      tmp += -self->k3[i] * (x[i] - xd[i]);
    }
    control->motorLeft_N = tmp + 9.81f*self->mass/2.0f;
    
    tmp = 0;
    for (int i = 0; i < 12; i++) {
      tmp += -self->k4[i] * (x[i] - xd[i]);
    }
    control->motorRight_N = tmp + 9.81f*self->mass/2.0f;
  }

  // logging
  leftMotor = control->motorLeft_N;
  rightMotor = control->motorRight_N;
  leftServo = control->servoLeft_deg - flappingAngleOffsetDeg; // log the servo value BEFORE flappingAngleOffsetDeg was added
  rightServo = control->servoRight_deg - flappingAngleOffsetDeg;
  flappingOffset = flappingAngleOffsetDeg;

  px = state->position.x;
  py = state->position.y;
  pz = state->position.z;

  roll = state->attitude.roll;
  pitch = -state->attitude.pitch;
  yaw = state->attitude.yaw;

  vx = state->velocity.x;
  vy = state->velocity.y;
  vz = state->velocity.z;

  wx = wx_avg;
  wy = wy_avg;
  wz = wz_avg;
  
  // disable motor output
  // control->motorLeft_N = 0.0f;
  // control->motorRight_N = 0.0f;
  // control->servoLeft_deg = 0.0f;
  // control->servoRight_deg = 10.0f;
}


void controllerLQRFirmwareInit(void)
{
  controllerLQRInit(&g_self);
}

bool controllerLQRFirmwareTest(void)
{
  return true;
}

void controllerLQRFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLQR(&g_self, control, setpoint, sensors, state, tick);
}

#include "log.h"
#include "param.h"
LOG_GROUP_START(ctrlLQR)

// log the full state (12 states)
LOG_ADD(LOG_FLOAT, px, &px)
LOG_ADD(LOG_FLOAT, py, &py)
LOG_ADD(LOG_FLOAT, pz, &pz)
LOG_ADD(LOG_FLOAT, vx, &vx)
LOG_ADD(LOG_FLOAT, vy, &vy)
LOG_ADD(LOG_FLOAT, vz, &vz)
LOG_ADD(LOG_FLOAT, wx, &wx)
LOG_ADD(LOG_FLOAT, wy, &wy)
LOG_ADD(LOG_FLOAT, wz, &wz)
LOG_ADD(LOG_FLOAT, roll, &roll)
LOG_ADD(LOG_FLOAT, pitch, &pitch)
LOG_ADD(LOG_FLOAT, yaw, &yaw)

// log the motor outputs
LOG_ADD(LOG_FLOAT, leftMotor, &leftMotor)
LOG_ADD(LOG_FLOAT, rightMotor, &rightMotor)
LOG_ADD(LOG_FLOAT, leftServo, &leftServo)
LOG_ADD(LOG_FLOAT, rightServo, &rightServo)

// log the flapping
LOG_ADD(LOG_FLOAT, offset, &flappingOffset)

// log the setpoints
LOG_ADD(LOG_FLOAT, sRoll, &sRoll)
LOG_ADD(LOG_FLOAT, sPitch, &sPitch)
LOG_ADD(LOG_FLOAT, sYaw, &sYaw)

LOG_ADD(LOG_FLOAT, sXPos, &sXPos)
LOG_ADD(LOG_FLOAT, sYPos, &sYPos)
LOG_ADD(LOG_FLOAT, sZPos, &sZPos)

LOG_ADD(LOG_FLOAT, sXVel, &sXVel)
LOG_ADD(LOG_FLOAT, sYVel, &sYVel)
LOG_ADD(LOG_FLOAT, sZVel, &sZVel)

// log the modes
LOG_ADD(LOG_UINT8, rollMode, &rollMode)
LOG_ADD(LOG_UINT8, pitchMode, &pitchMode)
LOG_ADD(LOG_UINT8, zMode, &zMode)

LOG_GROUP_STOP(ctrlLQR)

PARAM_GROUP_START(ctrlLQR)
PARAM_ADD(PARAM_UINT8, lqr_mode, &lqr_mode)
PARAM_ADD(PARAM_UINT8, flap_mode, &flappingConfig.state)
PARAM_ADD(PARAM_FLOAT, flap_hz, &flappingConfig.hz)
PARAM_ADD(PARAM_FLOAT, flap_a, &flappingConfig.amplitudeDeg)
PARAM_GROUP_STOP(ctrlLQR)