#include <math.h>
#include <string.h>

#include "math3d.h"
#include "controller_lqr.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

#include "debug.h"
#include "config.h"

// extern const unsigned int fh_lqr_max_index;
// extern const float u0[];
// extern const float k0[];
// extern const float x0[];
// extern const float K[];
// float get_u0(unsigned int sample_index, unsigned int state_index) { return u0[4*sample_index + state_index]; }
// float get_k0(unsigned int sample_index, unsigned int state_index) { return k0[4*sample_index + state_index]; }
// float get_x0(unsigned int sample_index, unsigned int state_index) { return x0[12*sample_index + state_index]; }
// float get_K(unsigned int sample_index, unsigned int row, unsigned int col) { return K[48*sample_index + 12*row + col]; }

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

// 1. try the higher mass value (works)
// 2. tune the thrusters so that z error is minimal (did this)
// 3. add in the bessel function multiplier for flapping (it actually over-compensates a bit)
static controllerLQR_t g_self = {
// use for paper?
  .k1 = {0.09149180f, -0.00669075f, 0.00000000f,
         0.04788407f, 0.57753220f, -0.21134638f, 0.13835939f, -0.01048333f, 0.00000000f, 0.00727996f, 0.06230944f, -0.04034646f},

  .k2 = {0.09149180f, 0.00669075f, -0.00000000f,
         -0.04788407f, 0.57753220f, 0.21134638f, 0.13835939f, 0.01048333f, -0.00000000f, -0.00727996f, 0.06230944f, 0.04034646f},

  .k3 = {0.00000000f, -0.63608557f, 2.18637896f,
         3.89758520f, 0.00000000f, 0.10493207f, 0.00000000f, -0.95396899f, 1.34283501f, 0.39255041f, 0.00000000f, 0.01327680f},

  .k4 = {-0.00000000f, 0.63608557f, 2.18637896f,
         -3.89758520f, -0.00000000f, -0.10493207f, -0.00000000f, 0.95396899f, 1.34283501f, -0.39255041f, -0.00000000f, -0.01327680f},
    .mass = 0.593f
  // .mass = 0.60610744f
};

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

// flapping parameters
struct flappingConfig_s {
    enum flappingMode_s {
      disabled = 0,
      enabled = 1,
    } state;
    float hz;
    float amplitudeDeg;
};

struct flappingConfig_s flappingConfig1 = {
  .state = disabled,
  .hz = 5,
  .amplitudeDeg = 20,
};

struct flappingConfig_s flappingConfig2 = {
  .state = disabled,
  .hz = 5,
  .amplitudeDeg = 20,
};

// 1/J_0(a*pi/180) where J_0 is the 0th order Bessel function of the first kind
// precomputed for a = 0, 1, 2, ..., 60 Deg
// This is used to account for the loss of cycle-averaged thrust due to servo oscillations
static float besselMultiplier[] = {
        1.000000f, // 0 deg
        1.000080f, // 1 deg
        1.000300f, // 2 deg
        1.000690f, // 3 deg
        1.001220f, // 4 deg
        1.001910f, // 5 deg
        1.002750f, // 6 deg
        1.003740f, // 7 deg
        1.004890f, // 8 deg
        1.006200f, // 9 deg
        1.007660f, // 10 deg
        1.009280f, // 11 deg
        1.011060f, // 12 deg
        1.013000f, // 13 deg
        1.015100f, // 14 deg
        1.017360f, // 15 deg
        1.019780f, // 16 deg
        1.022380f, // 17 deg
        1.025140f, // 18 deg
        1.028070f, // 19 deg
        1.031170f, // 20 deg
        1.034450f, // 21 deg
        1.037900f, // 22 deg
        1.041540f, // 23 deg
        1.045350f, // 24 deg
        1.049350f, // 25 deg
        1.053540f, // 26 deg
        1.057920f, // 27 deg
        1.062500f, // 28 deg
        1.067270f, // 29 deg
        1.072240f, // 30 deg
        1.077420f, // 31 deg
        1.082810f, // 32 deg
        1.088410f, // 33 deg
        1.094230f, // 34 deg
        1.100270f, // 35 deg
        1.106550f, // 36 deg
        1.113050f, // 37 deg
        1.119800f, // 38 deg
        1.126790f, // 39 deg
        1.134020f, // 40 deg
        1.141520f, // 41 deg
        1.149280f, // 42 deg
        1.157310f, // 43 deg
        1.165620f, // 44 deg
        1.174220f, // 45 deg
        1.183100f, // 46 deg
        1.192290f, // 47 deg
        1.201800f, // 48 deg
        1.211620f, // 49 deg
        1.221770f, // 50 deg
        1.232260f, // 51 deg
        1.243100f, // 52 deg
        1.254300f, // 53 deg
        1.265880f, // 54 deg
        1.277850f, // 55 deg
        1.290210f, // 56 deg
        1.302990f, // 57 deg
        1.316200f, // 58 deg
        1.329850f, // 59 deg
        1.343960f, // 60 deg
};

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
// the averagingFilter_s struct is updated every tick (1 ms), 5 Hz -> 200 ms, 10 Hz -> 100 ms, 15 Hz -> 50 ms
#define AVERAGING_FILTER_LENGTH 200
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

static void updateAveragingFilter(const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
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

static inline float safeInterpolate(float startValue, float endValue, float duration, float time) {
  if (time < 0) return startValue;
  if (time >= duration) return endValue;
  return startValue + (endValue-startValue)*time/duration;
}

static inline bool isClose(float a, float b) {
  return fabs(a-b) < 0.1;
}

/*
As the servo movements become faster, they no longer follow
perfect sinusoidal motion. In fact, the servo motion comes 
triangular because of the 1000 deg/s maximum speed limit.
As a result, the cycle-averaged thrust loss due to the servo
angles is slightly less than that for a perfect sinusoid.
We had to manually decrease these values of beta so that \Phi(beta)
would be a good compensation for different (w_hz, a_deg).
Without the manual adjustment, the bicopter will increase
its z height as it flaps.
*/
static int betaLUT(float w_hz, float a_deg) {
  int beta = 0;
  if (isClose(w_hz, 5.0f) && isClose(a_deg, 5.0f)) {
    beta = 15;
  }
  else if (isClose(w_hz, 5.0f) && isClose(a_deg, 10.0f)) {
    beta = 23;
  }
  else if (isClose(w_hz, 5.0f) && isClose(a_deg, 15.0f)) {
    beta = 32;
  }
  else if (isClose(w_hz, 5.0f) && isClose(a_deg, 20.0f)) {
    // not used anymore
    beta = 42;
  }
  else if (isClose(w_hz, 10.0f) && isClose(a_deg, 5.0f)) {
    beta = 5;
  }
  else if (isClose(w_hz, 10.0f) && isClose(a_deg, 10.0f)) {
    beta = 10;
  }
  else if (isClose(w_hz, 10.0f) && isClose(a_deg, 15.0f)) {
    beta = 15;
  }
  else if (isClose(w_hz, 15.0f) && isClose(a_deg, 5.0f)) {
    beta = 2;
  }
  else if (isClose(w_hz, 15.0f) && isClose(a_deg, 10.0f)) {
    beta = 5;
  }
  else if (isClose(w_hz, 15.0f) && isClose(a_deg, 15.0f)) {
    beta = 7;
  }
  return beta;
}

// controller runs at 100 Hz (should only be called every 10 ms)
float controlHelper(controllerLQR_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick,
                                         const struct flappingConfig_s *flapConfig)
{
  // forcing function produces flappingAngleOffsetDeg and thrustOffsetN.
  // thrustOffsetN will be mg/(2J(a)) where a is the amplitude of the servo command signal, which
  // varies based the flappingConfig.hz and the flappingConfig.amplitudeDeg. We use a lookup table
  // to determine what the value of a based on flappingConfig.hz and flappingConfig.amplitudeDeg.
  float flappingAngleOffsetDeg = 0.0f;
  float thrustOffsetN = self->mass * 9.81f / 2.0f;
  if (flapConfig->state == enabled) {
    flappingAngleOffsetDeg = flapConfig->amplitudeDeg * sinf(2*(float)M_PI*flapConfig->hz*tick/1000.0f);
    int beta = betaLUT(flapConfig->hz, flapConfig->amplitudeDeg);
    thrustOffsetN = (self->mass * 9.81f / 2.0f) * besselMultiplier[beta]; // comment out this line to set Phi = 0
  }

  // current state
  float x[12] = {state->position.x, state->position.y, state->position.z,
                 radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw),
                 state->velocity.x, state->velocity.y, state->velocity.z,
                 radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)};

  if (flapConfig->state == enabled) {
    // x[6] = 0.0f; // ignore the x velocity (significantly corrupted due to constant offset in x-direction)

    // use the cycle-averaged x position and x velocity to deal with the fact
    // that the IMU is located above the center of mass. This approximation
    // better represents the position and velocity of the CoM
    // x[0] = averagingFilter.avgX;
    // x[6] = averagingFilter.avgVx;
  }
  
  // desired state (by default, set only the desired position)
  float xd[12] = {0};
  xd[0] = setpoint->position.x;
  xd[1] = setpoint->position.y;
  xd[2] = setpoint->position.z;


  // Velocity control mode using the XBox 360 controller
  // althold flight mode. Values set with commander.send_setpoint()
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

  // implement the LQR control law (with flapping, if enabled)
  // u = -K(x - x_desired) + ue + u_delta[k]
  float tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -self->k1[i] * (x[i] - xd[i]);
  }
  control->servoLeft_deg = degrees(tmp) + flappingAngleOffsetDeg;

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -self->k2[i] * (x[i] - xd[i]);
  }
  control->servoRight_deg = degrees(tmp) + flappingAngleOffsetDeg;

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

  return flappingAngleOffsetDeg;
}

uint32_t startTick = 0;
enum LQRControllerState_t {
  hovering = 0,
  waiting,
  flapping
} lqrControllerState;

// #define TABLE3

void controllerLQR(controllerLQR_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  // runs at 1 kHz
  // updateAveragingFilter(sensors, state, tick);

  if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    return;
  }

  // controller runs at two different frequencies
  control->controlMode = controlModeLQR;

  // switch from waiting to flapping at whole second increments
  if (lqrControllerState == waiting && tick % 1000 == 0) {
    lqrControllerState = flapping;
    startTick = tick;
  }
  // how much time as passed since we entered the flapping state
  uint32_t elapsedTime = tick - startTick;

  // whichever flapping controller we use must report flappingAngleOffetDeg
  float flappingAngleOffsetDeg = 0.0f;

// this is the standard controller
#ifndef TABLE3
  if (lqrControllerState == hovering || lqrControllerState == waiting) {
      flappingConfig1.state = disabled;
      flappingConfig2.state = disabled;
      flappingAngleOffsetDeg = controlHelper(self, control, setpoint, sensors, state, tick, &flappingConfig1);
  } else {
    // flapping
    flappingConfig1.state = disabled;
    flappingConfig2.state = enabled;

    // u1 is hovering control, u2 is flapping control
    control_t u1, u2; 
    float offset1 = controlHelper(self, &u1, setpoint, sensors, state, elapsedTime, &flappingConfig1);
    float offset2 = controlHelper(self, &u2, setpoint, sensors, state, elapsedTime, &flappingConfig2);

    // as time increases, switch from u1 control (hovering) to u2 control (flapping)
    float lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime);
    control->servoLeft_deg = (1-lambda) * u1.servoLeft_deg + lambda * u2.servoLeft_deg;
    control->servoRight_deg = (1-lambda) * u1.servoRight_deg + lambda * u2.servoRight_deg;
    control->motorLeft_N = (1-lambda) * u1.motorLeft_N + lambda * u2.motorLeft_N;
    control->motorRight_N = (1-lambda) * u1.motorRight_N + lambda * u2.motorRight_N;
    flappingAngleOffsetDeg = (1-lambda) * offset1 + lambda * offset2;
  }
#endif

// force the firmware to do a pre-planned flapping pattern for table 3
#ifdef TABLE3
  if (lqrControllerState == hovering || lqrControllerState == waiting) {
    flappingConfig1.state = disabled;
    flappingConfig2.state = disabled;
    flappingAngleOffsetDeg = controlHelper(self, control, setpoint, sensors, state, tick, &flappingConfig1);
  } else {
    // flapping
    float lambda = 0.5f;
    if (elapsedTime < 5000) {
      // transition
      flappingConfig1.state = disabled;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 5.0f;
      flappingConfig2.amplitudeDeg = 5.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime);
    }
    else if (elapsedTime < 10000) {
      // 5 Hz, 10 deg
      flappingConfig1.state = enabled;
      flappingConfig1.hz = 5.0f;
      flappingConfig1.amplitudeDeg = 5.0f;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 5.0f;
      flappingConfig2.amplitudeDeg = 10.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 5000);
    }
    else if (elapsedTime < 15000) {
      // 5 Hz, 15 deg
      flappingConfig1.state = enabled;
      flappingConfig1.hz = 5.0f;
      flappingConfig1.amplitudeDeg = 10.0f;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 5.0f;
      flappingConfig2.amplitudeDeg = 15.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 10000);
    }
    // else if (elapsedTime < 20000) {
    //   // 5 Hz, 20 deg
    //   flappingConfig1.state = enabled;
    //   flappingConfig1.hz = 5.0f;
    //   flappingConfig1.amplitudeDeg = 15.0f;

    //   flappingConfig2.state = enabled;
    //   flappingConfig2.hz = 5.0f;
    //   flappingConfig2.amplitudeDeg = 20.0f;

    //   lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 15000);
    // }
    else if (elapsedTime < 20000){
      // stop flapping
      flappingConfig1.state = disabled;
      flappingConfig2.state = disabled;
    }
    else if (elapsedTime < 25000) {
      // 10 Hz, 5 deg
      flappingConfig1.state = disabled;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 10.0f;
      flappingConfig2.amplitudeDeg = 5.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 20000);
    }
    else if (elapsedTime < 30000) {
      // 10 Hz, 10 deg
      flappingConfig1.state = enabled;
      flappingConfig1.hz = 10.0f;
      flappingConfig1.amplitudeDeg = 5.0f;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 10.0f;
      flappingConfig2.amplitudeDeg = 10.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 25000);
    }
    else if (elapsedTime < 35000) {
      // 10 Hz, 15 deg
      flappingConfig1.state = enabled;
      flappingConfig1.hz = 10.0f;
      flappingConfig1.amplitudeDeg = 10.0f;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 10.0f;
      flappingConfig2.amplitudeDeg = 15.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 30000);
    }
    else if (elapsedTime < 40000) {
      // stop flapping
      flappingConfig1.state = disabled;
      flappingConfig2.state = disabled;
    }
    else if (elapsedTime < 45000) {
      // 15 Hz, 5 deg
      flappingConfig1.state = disabled;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 15.0f;
      flappingConfig2.amplitudeDeg = 5.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 40000);
    }
    else if (elapsedTime < 50000) {
      // 15 Hz, 10 deg
      flappingConfig1.state = enabled;
      flappingConfig1.hz = 15.0f;
      flappingConfig1.amplitudeDeg = 5.0f;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 15.0f;
      flappingConfig2.amplitudeDeg = 10.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 45000);
    }
    else if (elapsedTime < 55000) {
      // 15 Hz, 15 deg
      flappingConfig1.state = enabled;
      flappingConfig1.hz = 15.0f;
      flappingConfig1.amplitudeDeg = 10.0f;

      flappingConfig2.state = enabled;
      flappingConfig2.hz = 15.0f;
      flappingConfig2.amplitudeDeg = 15.0f;

      lambda = safeInterpolate(0.0f, 1.0f, (float) RAMP_TIME_MS, (float) elapsedTime - 50000);
    }
    else {
      // stop flapping
      flappingConfig1.state = disabled;
      flappingConfig2.state = disabled;
      lqrControllerState = hovering; // will take effect next time through function
    }

    control_t u1, u2; 
    float offset1 = controlHelper(self, &u1, setpoint, sensors, state, elapsedTime, &flappingConfig1);
    float offset2 = controlHelper(self, &u2, setpoint, sensors, state, elapsedTime, &flappingConfig2);

    // as time increases, switch from u1 control (hovering) to u2 control (flapping)
    control->servoLeft_deg = (1-lambda) * u1.servoLeft_deg + lambda * u2.servoLeft_deg;
    control->servoRight_deg = (1-lambda) * u1.servoRight_deg + lambda * u2.servoRight_deg;
    control->motorLeft_N = (1-lambda) * u1.motorLeft_N + lambda * u2.motorLeft_N;
    control->motorRight_N = (1-lambda) * u1.motorRight_N + lambda * u2.motorRight_N;
    flappingAngleOffsetDeg = (1-lambda) * offset1 + lambda * offset2;
  }
#endif // TABLE3

  // logging
  leftServo = control->servoLeft_deg;
  rightServo = control->servoRight_deg;
  leftMotor = control->motorLeft_N;
  rightMotor = control->motorRight_N;
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

LOG_ADD(LOG_UINT8, flap_mode, &lqrControllerState)
LOG_ADD(LOG_UINT8, flap_mode1, &flappingConfig1.state)
LOG_ADD(LOG_UINT8, flap_mode2, &flappingConfig2.state)
LOG_ADD(LOG_UINT32, tick, &lastTick)

LOG_GROUP_STOP(ctrlLQR)

PARAM_GROUP_START(ctrlLQR)
PARAM_ADD(PARAM_UINT8, flap_mode, &lqrControllerState)
PARAM_ADD(PARAM_FLOAT, flap_hz, &flappingConfig2.hz)
PARAM_ADD(PARAM_FLOAT, flap_a, &flappingConfig2.amplitudeDeg)
PARAM_GROUP_STOP(ctrlLQR)