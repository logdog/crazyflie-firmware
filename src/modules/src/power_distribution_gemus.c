/**
 *  _   _                     _           _
 * | | | |                   | |         | |
 * | |_| |_   _ _   _ _ __   | |     __ _| |__
 * |  _  | | | | | | | '_ \  | |    / _` | '_ \
 * | | | | |_| | |_| | | | | | |___| (_| | |_) |
 * \_| |_/\__, |\__,_|_| |_| \_____/\__,_|_.__/
 *         __/ |
 *        |___/
 *
 * @file power_distribution_gemus.c
 * @author Logan Dihel
* @brief Tells the motors what to do given the thrust and torque controller outputs
 * @details This file was modified from power_distribution_bicopter.c
 */
#include "power_distribution.h"

#include <string.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"
#include "bicopterdeck.h"
#include "pm.h"
#include "math3d.h"
#include "motors.h"
#include "supervisor.h"  // check if we are in e-stop mode


const int zeroPosition_us = 1500;
struct servoConfig_s {
    int16_t trim;       // sets the zero position (trim=0 <--> 1500 us pulse)
    int8_t sign;        // flips positive direction (should be +1 or -1)
    float usPerDeg;     // pulse length per degree
    int16_t min_us;     // minimum pulse length (us)
    int16_t max_us;     // maximum pulse length (us)
};

// calcualte the pulse length in microseconds
uint16_t degToMicroseconds(struct servoConfig_s servo, float deg) {
    return zeroPosition_us + servo.trim + servo.sign * servo.usPerDeg * deg;
}

struct gemusConfig_s {
    struct servoConfig_s servo1;
    struct servoConfig_s servo2;
    struct servoConfig_s servo3;
    struct servoConfig_s servo4;
};

struct gemusConfig_s gemusConfig = {
    .servo1 = {.trim=0, .sign=1, .usPerDeg=11.11f, .min_us=800, .max_us=2200},
    .servo2 = {.trim=0, .sign=-1, .usPerDeg=11.11f, .min_us=800, .max_us=2200},
    .servo3 = {.trim=0, .sign=1, .usPerDeg=11.11f, .min_us=800, .max_us=2200},
    .servo4 = {.trim=0, .sign=1, .usPerDeg=11.11f, .min_us=800, .max_us=2200}
};

#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0)) && defined(CONFIG_MOTORS_DEFAULT_IDLE_THRUST) && (CONFIG_MOTORS_DEFAULT_IDLE_THRUST > 0)
#error "CONFIG_MOTORS_REQUIRE_ARMING must be defined and not set to 0 if CONFIG_MOTORS_DEFAULT_IDLE_THRUST is greater than 0"
#endif
#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
static uint16_t maxThrust = UINT16_MAX;

int powerDistributionMotorType(uint32_t id)
{
    return 0; // all motors are servos, so they do not use "thrust"
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
    uint16_t stopValue = zeroPosition_us;
    switch (id) {
        case MOTOR_M1:  stopValue = degToMicroseconds(gemusConfig.servo1, 0.0f); break;
        case MOTOR_M2:  stopValue = degToMicroseconds(gemusConfig.servo2, 0.0f); break;
        case MOTOR_M3:  stopValue = degToMicroseconds(gemusConfig.servo3, 0.0f); break;
        case MOTOR_M4:  stopValue = degToMicroseconds(gemusConfig.servo4, 0.0f); break;
        default: break;
    }
    return stopValue;
}

void powerDistributionInit(void)
{
#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
  if(idleThrust > 0) {
        DEBUG_PRINT("WARNING: idle thrust will be overridden with value 0. Autoarming can not be on while idle thrust is higher than 0. If you want to use idle thust please use use arming\n");
    }
#endif
}

bool powerDistributionTest(void)
{
    bool pass = true;
    return pass;
}

// these are used as log variables
static uint16_t servo1us, servo2us, servo3us, servo4us;

static void powerDistribution4Servos(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {

    servo1us = degToMicroseconds(gemusConfig.servo1, control->servo1_deg);
    servo2us = degToMicroseconds(gemusConfig.servo2, control->servo2_deg);
    servo3us = degToMicroseconds(gemusConfig.servo3, control->servo3_deg);
    servo4us = degToMicroseconds(gemusConfig.servo4, control->servo4_deg);

    motorThrustUncapped->motors.m1 = servo1us;
    motorThrustUncapped->motors.m2 = servo2us;
    motorThrustUncapped->motors.m3 = servo3us;
    motorThrustUncapped->motors.m4 = servo4us;
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlMode4Servos:
        powerDistribution4Servos(control, motorThrustUncapped);
        break;
    default:
        // Nothing here
        break;
    }
}

uint16_t limitThrust(int32_t value, int32_t min, int32_t max, bool* isCapped)
{
  if (value < min) {
    *isCapped = true;
    return min;
  }
  if (value > max) {
    *isCapped = true;
    return max;
  }
  return value;
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
    bool isCapped = false;

    motorPwm->motors.m1 = limitThrust(motorThrustBatCompUncapped->motors.m1, gemusConfig.servo1.min_us, gemusConfig.servo1.max_us, &isCapped);
    motorPwm->motors.m2 = limitThrust(motorThrustBatCompUncapped->motors.m2, gemusConfig.servo2.min_us, gemusConfig.servo2.max_us, &isCapped);
    motorPwm->motors.m3 = limitThrust(motorThrustBatCompUncapped->motors.m3, gemusConfig.servo3.min_us, gemusConfig.servo3.max_us, &isCapped);
    motorPwm->motors.m4 = limitThrust(motorThrustBatCompUncapped->motors.m4, gemusConfig.servo4.min_us, gemusConfig.servo4.max_us, &isCapped);

    return isCapped;
}

uint32_t powerDistributionGetIdleThrust()
{
    int32_t thrust = idleThrust;
#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
    thrust = 0;
#endif
    return thrust;
}

float powerDistributionGetMaxThrust() {
    // maximum thrust per motor (70% pwm at nominal voltage in Newtons)
    return 5.0f;
}

LOG_GROUP_START(powerDist)

/**
 * @brief servo PWM in microseconds
 */
LOG_ADD(LOG_UINT16, servo1us, &servo1us)
LOG_ADD(LOG_UINT16, servo2us, &servo2us)
LOG_ADD(LOG_UINT16, servo3us, &servo3us)
LOG_ADD(LOG_UINT16, servo4us, &servo4us)

LOG_GROUP_STOP(powerDist)

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &maxThrust)

// set the trim or sign values via a parameter update
PARAM_ADD(PARAM_INT16, servo1trim, &gemusConfig.servo1.trim)
PARAM_ADD(PARAM_INT16, servo2trim, &gemusConfig.servo2.trim)
PARAM_ADD(PARAM_INT16, servo3trim, &gemusConfig.servo3.trim)
PARAM_ADD(PARAM_INT16, servo4trim, &gemusConfig.servo4.trim)
PARAM_ADD(PARAM_INT8,  servo1sign, &gemusConfig.servo1.sign)
PARAM_ADD(PARAM_INT8,  servo2sign, &gemusConfig.servo2.sign)
PARAM_ADD(PARAM_INT8,  servo3sign, &gemusConfig.servo3.sign)
PARAM_ADD(PARAM_INT8,  servo4sign, &gemusConfig.servo4.sign)
PARAM_GROUP_STOP(powerDist)