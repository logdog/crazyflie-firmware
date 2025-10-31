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

// controller works very well, especially if you set k1[10] = k2[10] = 0.10
// static controllerLQR_t g_self = {
//   .k1 = {0.09149180f, -0.00926999f, 0.00000000f,
//          0.04898391f, 0.57753220f, -0.21136620f, 0.13835939f, -0.01162586f, 0.00000000f, 0.00733346f, 0.06230944f, -0.04034881f},

//   .k2 = {0.09149180f, 0.00926999f, -0.00000000f,
//          -0.04898391f, 0.57753220f, 0.21136620f, 0.13835939f, 0.01162586f, 0.00000000f, -0.00733346f, 0.06230944f, 0.04034881f},

//   .k3 = {0.00000000f, -0.89836564f, 2.14498959f,
//          4.00516745f, 0.00000000f, 0.10302765f, 0.00000000f, -1.06635256f, 2.42921273f, 0.39774998f, 0.00000000f, 0.01305347f},

//   .k4 = {0.00000000f, 0.89836564f, 2.14498959f,
//          -4.00516745f, 0.00000000f, -0.10302765f, 0.00000000f, 1.06635256f, 2.42921273f, -0.39774998f, 0.00000000f, -0.01305347f},

//   .mass = 0.575f
// };

// 1. try the higher mass value (works)
// 2. tune the thrusters so that z error is minimal (did this)
// 3. add in the bessel function multiplier for flapping (it actually over-compensates a bit)
static controllerLQR_t g_self = {
  .k1 = {0.09149180f, -0.00926999f, 0.00000000f,
         0.04898391f, 0.57753220f, -0.21136620f, 0.13835939f, -0.01162586f, 0.00000000f, 0.00733346f, 0.06230944f, -0.04034881f},

  .k2 = {0.09149180f, 0.00926999f, -0.00000000f,
         -0.04898391f, 0.57753220f, 0.21136620f, 0.13835939f, 0.01162586f, 0.00000000f, -0.00733346f, 0.06230944f, 0.04034881f},

  .k3 = {0.00000000f, -0.89836564f, 2.14498959f,
         4.00516745f, 0.00000000f, 0.10302765f, 0.00000000f, -1.06635256f, 2.42921273f, 0.39774998f, 0.00000000f, 0.01305347f},

  .k4 = {0.00000000f, 0.89836564f, 2.14498959f,
         -4.00516745f, 0.00000000f, -0.10302765f, 0.00000000f, 1.06635256f, 2.42921273f, -0.39774998f, 0.00000000f, -0.01305347f},

  .mass = 0.60610744f
};

// static controllerLQR_t g_self = {
//   .k1 = {0.09149180f, -0.00926999f, 0.00000000f,
//          0.04898391f, 0.57753220f, -0.21136620f, 0.13835939f, -0.01162586f, 0.00000000f, 0.00733346f, 0.1f, -0.04034881f},

//   .k2 = {0.09149180f, 0.00926999f, -0.00000000f,
//          -0.04898391f, 0.57753220f, 0.21136620f, 0.13835939f, 0.01162586f, 0.00000000f, -0.00733346f, 0.1f, 0.04034881f},

//   .k3 = {0.00000000f, -0.89836564f, 2.14498959f,
//          4.00516745f, 0.00000000f, 0.10302765f, 0.00000000f, -1.06635256f, 2.42921273f, 0.39774998f, 0.00000000f, 0.01305347f},

//   .k4 = {0.00000000f, 0.89836564f, 2.14498959f,
//          -4.00516745f, 0.00000000f, -0.10302765f, 0.00000000f, 1.06635256f, 2.42921273f, -0.39774998f, 0.00000000f, -0.01305347f},

//   .mass = 0.60610744f
// };

// also try the following one:

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
      enabled = 3,
      waitToDisable = 4,
      
      // the demos for 5hz, 10hz, 15hz
      waitToDemo = 5,
      demo5,
      demo10,
      demo15

    } state;
    float hz;
    float amplitudeDeg;
    uint32_t lastTick;
    uint8_t useAveragingFilter;
};

struct flappingConfig_s flappingConfig = {
  .state = disabled,
  .hz = 10,
  .amplitudeDeg = 10,
  .lastTick = 0,
  .useAveragingFilter = 0,
};

// 1/J_0(a*pi/180) where J_0 is the 0th order Bessel function of the first kind
// precomputed for a = 0, 1, 2, ..., 30 Deg
// This is used to account for the loss of cycle-averaged thrust due to servo oscillations
static float besselMultiplier[] = {
  1.0f,         // 0 deg
  1.00010001f,
  1.00030009f,
  1.00070049f,
  1.001201442f,
  1.001903617f,
  1.00270731f,
  1.003713741f,
  1.004924128f,
  1.00623868f,
  1.007658202f,
  1.009285426f,
  1.011020119f,
  1.012965964f,
  1.015125368f,
  1.017397497f,
  1.019783806f,
  1.022390349f,
  1.025115325f,
  1.028066207f,
  1.031140441f,
  1.034447088f,
  1.037882719f,
  1.041558171f,
  1.045369015f,
  1.049317943f,
  1.053518753f,
  1.057977148f,
  1.062473438f,
  1.067235859f,
  1.07227107f,  // 30 deg
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

static uint32_t lastTick;

// we don't need to store the whole array - just accumuate and then divide
// the averagingFilter_s struct is updated every tick (1 ms), so 1 flapping cycle is 100 ticks (100 ms)
#define AVERAGING_FILTER_LENGTH 100
struct averagingFilter_s {
  float pitch[AVERAGING_FILTER_LENGTH];
  float x[AVERAGING_FILTER_LENGTH];
  float wy[AVERAGING_FILTER_LENGTH];
  float vx[AVERAGING_FILTER_LENGTH];

  // warning: pitch is in degrees, wy in deg/s
  float accPitch, accX, accWy, accVx; // accumulated
  float avgPitch, avgX, avgWy, avgVx; // average
  bool hasFilledUpOnce;
  uint32_t count;
};

struct averagingFilter_s averagingFilter = {
  .accPitch = 0.0f,
  .accX = 0.0f,
  .accWy = 0.0f,
  .accVx = 0.0f,
  .avgPitch = 0.0f,
  .avgX = 0.0f,
  .avgWy = 0.0f,
  .avgVx = 0.0f,
  .hasFilledUpOnce = false,
  .count = 0,
};

#define RAMP_TIME_MS (1000)

// counter variables
// static unsigned int lqr_count = 0;
// static unsigned int fh_lqr_count = 0;

void updateAveragingFilter(const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // if we havent filled up the buffer all the way ever before, the average will be wrong but that's ok
  if (!averagingFilter.hasFilledUpOnce) {

    // record the data to an array
    averagingFilter.pitch[averagingFilter.count] = -state->attitude.pitch;
    averagingFilter.x[averagingFilter.count] = state->position.x;
    averagingFilter.wy[averagingFilter.count] = sensors->gyro.y;
    averagingFilter.vx[averagingFilter.count] = state->velocity.x;

    // update the accumulator
    averagingFilter.accPitch += averagingFilter.pitch[averagingFilter.count];
    averagingFilter.accX += averagingFilter.x[averagingFilter.count];
    averagingFilter.accWy += averagingFilter.wy[averagingFilter.count];
    averagingFilter.accVx += averagingFilter.vx[averagingFilter.count];

    // update the counter
    averagingFilter.count++;

    // take the average
    averagingFilter.avgPitch = averagingFilter.accPitch / (float) averagingFilter.count;
    averagingFilter.avgX = averagingFilter.accX / (float) averagingFilter.count;
    averagingFilter.avgWy = averagingFilter.accWy / (float) averagingFilter.count;
    averagingFilter.avgVx = averagingFilter.accVx / (float) averagingFilter.count;

    if (averagingFilter.count >= AVERAGING_FILTER_LENGTH) {
      averagingFilter.count %= AVERAGING_FILTER_LENGTH;
      averagingFilter.hasFilledUpOnce = true;
    }
  }
  else {
    // if we have a full buffer, then each iteration we can update the moving average

    // update the accumulator by first subtracting the oldest read data
    // then adding in the current measurements

    // decrease accumulator
    averagingFilter.accPitch -= averagingFilter.pitch[averagingFilter.count];
    averagingFilter.accX -= averagingFilter.x[averagingFilter.count];
    averagingFilter.accWy -= averagingFilter.wy[averagingFilter.count];
    averagingFilter.accVx -= averagingFilter.vx[averagingFilter.count];

    // record data to array
    averagingFilter.pitch[averagingFilter.count] = -state->attitude.pitch;
    averagingFilter.x[averagingFilter.count] = state->position.x;
    averagingFilter.wy[averagingFilter.count] = sensors->gyro.y;
    averagingFilter.vx[averagingFilter.count] = state->velocity.x;

    // increase accumulator
    averagingFilter.accPitch += averagingFilter.pitch[averagingFilter.count];
    averagingFilter.accX += averagingFilter.x[averagingFilter.count];
    averagingFilter.accWy += averagingFilter.wy[averagingFilter.count];
    averagingFilter.accVx += averagingFilter.vx[averagingFilter.count];


    // update the counter
    averagingFilter.count++;
    averagingFilter.count %= AVERAGING_FILTER_LENGTH;

  }
  // we will only calculate the average when we actually need the control input (saves us a lot of calculations)
  if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    averagingFilter.avgPitch = averagingFilter.accPitch / (float) AVERAGING_FILTER_LENGTH; // deg
    averagingFilter.avgX = averagingFilter.accX / (float) AVERAGING_FILTER_LENGTH;         // m
    averagingFilter.avgWy = averagingFilter.accWy / (float) AVERAGING_FILTER_LENGTH;       // deg/s
    averagingFilter.avgVx = averagingFilter.accVx / (float) AVERAGING_FILTER_LENGTH;       // m/s
    return;
  }
}

inline float safeInterpolate(float startValue, float endValue, float duration, float time) {
  if (time < 0) return startValue;
  if (time >= duration) return endValue;
  return startValue + (endValue-startValue)*time/duration;
}


void controllerLQR(controllerLQR_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  control->controlMode = controlModeLQR;
  // runs at 1 kHz
  updateAveragingFilter(sensors, state, tick);

  // add the flapping control signal
  float flappingAngleOffsetDeg = 0.0f;
  float thrustOffsetN = self->mass * 9.81f / 2.0f;
  
  // State Machine Transition Rules:
  // 1. The flappingConfig.state will be set to "waitToEnable", "waitToDisable" (or "disable")
  //    via the parameter `ctrlLQR.flap_mode`
  // 2. Only transition to start/stop flapping at whole second increments
  // 3. The rampingUp state will last `RAMP_TIME_MS` milliseconds
  if (flappingConfig.state == waitToEnable && (tick % 1000) == 0) {
    flappingConfig.state = enabled;
    flappingConfig.lastTick = tick;
  }
  else if (flappingConfig.state == waitToDisable && (tick % 1000) == 0) {
    flappingConfig.state = disabled;
  }
  else if (flappingConfig.state == waitToDemo && (tick % 1000) == 0) {
    flappingConfig.state = demo5;
    flappingConfig.lastTick = tick;
  }

  if (flappingConfig.state == enabled) {
    int timeElapsed = tick - flappingConfig.lastTick;
    // ramp up for RAMP_TIME_MS and then remain at flappingConfig.amplitudeDeg
    float amplitude = safeInterpolate(0, flappingConfig.amplitudeDeg, RAMP_TIME_MS, timeElapsed);
    flappingAngleOffsetDeg = amplitude * sinf(2*(float)M_PI*flappingConfig.hz*timeElapsed/1000.0f);
  }

  // 5 Hz demo
  if (flappingConfig.state == demo5) {
    int timeElapsed = tick - flappingConfig.lastTick; // elapsed time in ms

    float amplitude = 0.0f;
    float hz = 5.0f;
    if (timeElapsed < 3000) {
      amplitude = safeInterpolate(0, 5.0f, RAMP_TIME_MS, timeElapsed);
    }
    else if (timeElapsed < 6000) {
      amplitude = safeInterpolate(5.0f, 10.0f, RAMP_TIME_MS, timeElapsed - 3000);
    }
    else if (timeElapsed < 9000) {
      amplitude = safeInterpolate(10.0f, 15.0f, RAMP_TIME_MS, timeElapsed - 6000);
    }
    else if (timeElapsed < 15000) {
      amplitude = safeInterpolate(15.0f, 20.0f, RAMP_TIME_MS, timeElapsed - 9000);
    }
    else if (timeElapsed < 20000) {
      amplitude = 0.0f;
    }
    else {
      flappingConfig.state = demo10;
      flappingConfig.lastTick = tick;
    }
    flappingAngleOffsetDeg = amplitude * sinf(2*(float)M_PI*hz*timeElapsed/1000.0f);
  }

  // 10 Hz demo
  if (flappingConfig.state == demo10) {
    int timeElapsed = tick - flappingConfig.lastTick; // elapsed time in ms

    float amplitude = 0.0f;
    float hz = 10.0f;
    if (timeElapsed < 5000) {
      amplitude = safeInterpolate(0, 5.0f, RAMP_TIME_MS, timeElapsed);
    }
    else if (timeElapsed < 1000) {
      amplitude = safeInterpolate(5.0f, 10.0f, RAMP_TIME_MS, timeElapsed - 5000);
    }
    else if (timeElapsed < 15000) {
      amplitude = safeInterpolate(10.0f, 15.0f, RAMP_TIME_MS, timeElapsed - 10000);
    }
    else if (timeElapsed < 20000) {
      amplitude = 0.0f;
    }
    else {
      flappingConfig.state = demo15;
      flappingConfig.lastTick = tick;
    }
    flappingAngleOffsetDeg = amplitude * sinf(2*(float)M_PI*hz*timeElapsed/1000.0f);
  }

  // 15 Hz demo
  if (flappingConfig.state == demo15) {
    int timeElapsed = tick - flappingConfig.lastTick; // elapsed time in ms

    float amplitude = 0.0f;
    float hz = 15.0f;
    if (timeElapsed < 5000) {
      amplitude = safeInterpolate(0, 5.0f, RAMP_TIME_MS, timeElapsed);
    }
    else if (timeElapsed < 10000) {
      amplitude = safeInterpolate(5.0f, 10.0f, RAMP_TIME_MS, timeElapsed - 5000);
    }
    else if (timeElapsed < 15000) {
      amplitude = 0.0f;
    }
    else {
      flappingConfig.state = disabled;
    }
    flappingAngleOffsetDeg = amplitude * sinf(2*(float)M_PI*hz*timeElapsed/1000.0f);

    //   uint8_t a_deg = (uint8_t) roundf(fabsf(multiplier * flappingConfig.amplitudeDeg));
    //   if (a_deg >= 0 && a_deg <= 30) { 
    //     thrustOffsetN = (self->mass * 9.81f / 2.0f) * besselMultiplier[a_deg]; 
    //   }
  }

  if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    // update the servos at 1kHz
    control->servoLeft_deg = flappingAngleOffsetDeg + lastServoLeftDeg;   // when not flapping: 0 deg
    control->servoRight_deg = flappingAngleOffsetDeg + lastServoRightDeg; // when not flapping: 0 deg
    return;
  }
  
  // LQR controller runs at 100 Hz

  // current state
  float x[12] = {state->position.x, state->position.y, state->position.z,
                 radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw),
                 state->velocity.x, state->velocity.y, state->velocity.z,
                 radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)};
  // x[9] = 0;
  // x[10] = 0;
  // x[11] = 0;

  // // when flapping, use cycle-averaged estimator, if enabled
  // if (flappingConfig.useAveragingFilter == 1) {
  //   // x[0] = averagingFilter.avgX;
  //   // x[4] = radians(averagingFilter.avgPitch);
  //   // x[6] = averagingFilter.avgVx;
  //   x[10] = radians(averagingFilter.avgWy);
  //   // x[10] = 0;

  //   // change the control gains to be more aggressive
  //   // self->k1[4] = 0.5;
  //   // self->k2[4] = 0.5;
  //   self->k1[10] = 0.10f;
  //   self->k2[10] = 0.10f;
  // }
  // else {
  //   // the original control signal
  //   self->k1[10] = 0.10f;
  //   self->k2[10] = 0.10f;
  //   // self->k1[10] = 0.06230944f;
  //   // self->k2[10] = 0.06230944f;
  // }


  // set the desired state, depending on the flight mode
  float xd[12] = {0};
  if (setpoint->mode.roll == modeAbs && setpoint->mode.pitch == modeAbs && setpoint->mode.z == modeVelocity) {
    // althold flight mode. Values set with commander.send_setpoint()
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

  // if (lqr_mode == FINITE_HORIZON && fh_lqr_count < fh_lqr_max_index) {
  // // if (false) {
  //   // FH LQR Controller
  //   // u = -K(t) (x - x0(t)) - k0(t) + u0(t)
  //   float tmp[4] = { 0.0f };

  //   for (int row = 0; row < 4; row++) {
  //     for (int i = 0; i < 12; i++) {
  //       tmp[row] += -get_K(fh_lqr_count, row, i) * (x[i] - get_x0(fh_lqr_count, i));
  //     }
  //     tmp[row] += -get_k0(fh_lqr_count, row);
  //     tmp[row] += get_u0(fh_lqr_count, row);
  //   }

  //   control->servoLeft_deg = degrees(tmp[0]);
  //   control->servoRight_deg = degrees(tmp[1]);
  //   control->motorLeft_N = tmp[2];
  //   control->motorRight_N = tmp[3];

  //   // update the counter to step through the trajectory
  //   fh_lqr_count++;
    
  //   // return to the standard infinite horizon LQR controller
  //   if (fh_lqr_count >= fh_lqr_max_index) {
  //     lqr_mode = DEFAULT;
  //     fh_lqr_count = 0;
  //   }
  // }


  // Original LQR Controller
  // u = -K(x - x_desired) + ue
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
  control->motorLeft_N = tmp + thrustOffsetN;
  
  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -self->k4[i] * (x[i] - xd[i]);
  }
  control->motorRight_N = tmp + thrustOffsetN;

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

  wx = sensors->gyro.x;
  wy = sensors->gyro.y;
  wz = sensors->gyro.z;

  lastTick = tick;
  
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

// log the filtered states
LOG_ADD(LOG_FLOAT, avgPitch, &averagingFilter.avgPitch)
LOG_ADD(LOG_FLOAT, avgWy, &averagingFilter.avgWy)
LOG_ADD(LOG_FLOAT, avgX, &averagingFilter.avgX)
LOG_ADD(LOG_FLOAT, avgVx, &averagingFilter.avgVx)

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

LOG_ADD(LOG_UINT8, flap_mode, &flappingConfig.state)
LOG_ADD(LOG_UINT8, flap_filter, &flappingConfig.useAveragingFilter)
LOG_ADD(LOG_UINT32, tick, &lastTick)

LOG_GROUP_STOP(ctrlLQR)

PARAM_GROUP_START(ctrlLQR)
PARAM_ADD(PARAM_UINT8, lqr_mode, &lqr_mode)
PARAM_ADD(PARAM_UINT8, flap_mode, &flappingConfig.state)
PARAM_ADD(PARAM_FLOAT, flap_hz, &flappingConfig.hz)
PARAM_ADD(PARAM_FLOAT, flap_a, &flappingConfig.amplitudeDeg)
PARAM_ADD(PARAM_UINT8, flap_filter, &flappingConfig.useAveragingFilter)
PARAM_GROUP_STOP(ctrlLQR)