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
 * @file power_distribution_beat.c
 * @author Logan Dihel
* @brief Tells the motors what to do given the thrust and torque controller outputs
 * @details This file was modified from power_distribution_gemus.c
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

struct bldcConfig_s {
    float pwmToThrustA; // 
    float pwmToThrustB;
    float trim; // force multiplier
};

struct beatConfig_s {
    struct servoConfig_s servo1; // phi_L
    struct servoConfig_s servo2; // psi_L
    struct servoConfig_s servo3; // phi_R
    struct servoConfig_s servo4; // psi_R
    struct bldcConfig_s bldc1;   // left motor
    struct bldcConfig_s bldc2;   // right motor
};

struct beatConfig_s beatConfig = {
    .servo1 = {.trim=-125, .sign=-1, .usPerDeg=11.11f, .min_us=800, .max_us=2200},
    .servo2 = {.trim=-90, .sign=1, .usPerDeg=11.11f, .min_us=800, .max_us=2200},
    .servo3 = {.trim=90, .sign=1, .usPerDeg=11.11f, .min_us=800, .max_us=2200},
    .servo4 = {.trim=60, .sign=-1, .usPerDeg=11.11f, .min_us=800, .max_us=2200},
    .bldc1  = {.pwmToThrustA = 0.05163731f, .pwmToThrustB = 0.32107592f, .trim=1.0f},
    .bldc2  = {.pwmToThrustA = 0.05163731f, .pwmToThrustB = 0.32107592f, .trim=1.0f}
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

// calcualte the pulse length in microseconds
uint16_t degToMicroseconds(struct servoConfig_s * servo, float deg) {
    return zeroPosition_us + servo->sign * (servo->usPerDeg * deg + servo->trim);
}

int powerDistributionMotorType(uint32_t id)
{
    return (id == 4 || id == 5); // brushless motors
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
    uint16_t stopValue;

    if (powerDistributionMotorType(id) == 0){
        stopValue = zeroPosition_us;
        // servo motor
        switch (id) {
            case 0:  stopValue = degToMicroseconds(&beatConfig.servo1, 0.0f); break;
            case 1:  stopValue = degToMicroseconds(&beatConfig.servo2, 0.0f); break;
            case 2:  stopValue = degToMicroseconds(&beatConfig.servo3, 0.0f); break;
            case 3:  stopValue = degToMicroseconds(&beatConfig.servo4, 0.0f); break;
            default: break;
        }
    }
    else {
        // brushless motor
        stopValue = 0;
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

static uint32_t motorThrustToDSHOT(struct bldcConfig_s *bldc, float motorThrust_N) {
    // given the desired force (N), get the DSHOT value to send to the motors.
    // motorThrustUncapped->motors.m1 is in range [0, UINT16_MAX] which is sent as a DSHOT value

    // Force (N) = pwmToThrustA * Veff^2 + pwmToThrustB * Veff
    float y = (-bldc->pwmToThrustB + sqrtf(bldc->pwmToThrustB * bldc->pwmToThrustB + 4.0f * bldc->pwmToThrustA * motorThrust_N)) / (2.0f * bldc->pwmToThrustA);
    
    #ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED
    float vBatt = pmGetBatteryVoltage();
    #else
    float vBatt = 14.8f; // 4S battery nominal voltage
    #endif

    float pwm = y / vBatt;

    // maximum pwm value is 1.0
    // pwmAdjust is a parameter we can set to scale up or down all thrusts
    // TODO: 
    float m_pwm = fmin(pwm, 1.0f);

    return m_pwm * UINT16_MAX;
}


static int counter = 0;
static void powerDistributionBeat(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    motorThrustUncapped->motors.s1 = degToMicroseconds(&beatConfig.servo1, control->phiLeft_deg);
    motorThrustUncapped->motors.s2 = degToMicroseconds(&beatConfig.servo2, control->psiLeft_deg);
    motorThrustUncapped->motors.s3 = degToMicroseconds(&beatConfig.servo3, control->phiRight_deg);
    motorThrustUncapped->motors.s4 = degToMicroseconds(&beatConfig.servo4, control->psiRight_deg);

    motorThrustUncapped->motors.m1 = motorThrustToDSHOT(&beatConfig.bldc1, control->thrustLeft_N);
    motorThrustUncapped->motors.m4 = motorThrustToDSHOT(&beatConfig.bldc2, control->thrustRight_N);

    // motorThrustUncapped->motors.s3 = 1700;

    if (counter++ % 250 == 0) {
        DEBUG_PRINT("servos: %d, %d, %d, %d\n", motorThrustUncapped->motors.s1, 
            motorThrustUncapped->motors.s2,
            motorThrustUncapped->motors.s3,
            motorThrustUncapped->motors.s4);
    }
}



void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeBeat:
        powerDistributionBeat(control, motorThrustUncapped);
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

    // Motors M1 and M4
    motorPwm->motors.m1 = limitThrust(motorThrustBatCompUncapped->motors.m1, idleThrust, maxThrust, &isCapped);
    motorPwm->motors.m4 = limitThrust(motorThrustBatCompUncapped->motors.m4, idleThrust, maxThrust, &isCapped);

    motorPwm->motors.s1 = limitThrust(motorThrustBatCompUncapped->motors.s1, beatConfig.servo1.min_us, beatConfig.servo1.max_us, &isCapped);
    motorPwm->motors.s2 = limitThrust(motorThrustBatCompUncapped->motors.s2, beatConfig.servo2.min_us, beatConfig.servo2.max_us, &isCapped);
    motorPwm->motors.s3 = limitThrust(motorThrustBatCompUncapped->motors.s3, beatConfig.servo3.min_us, beatConfig.servo3.max_us, &isCapped);
    motorPwm->motors.s4 = limitThrust(motorThrustBatCompUncapped->motors.s4, beatConfig.servo4.min_us, beatConfig.servo4.max_us, &isCapped);

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

// LOG_GROUP_START(powerDist)

// /**
//  * @brief servo PWM in microseconds
//  */
// LOG_ADD(LOG_UINT16, servo1us, &servo1us)
// LOG_ADD(LOG_UINT16, servo2us, &servo2us)
// LOG_ADD(LOG_UINT16, servo3us, &servo3us)
// LOG_ADD(LOG_UINT16, servo4us, &servo4us)

// LOG_GROUP_STOP(powerDist)

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
PARAM_ADD(PARAM_INT16, servo1trim, &beatConfig.servo1.trim)
PARAM_ADD(PARAM_INT16, servo2trim, &beatConfig.servo2.trim)
PARAM_ADD(PARAM_INT16, servo3trim, &beatConfig.servo3.trim)
PARAM_ADD(PARAM_INT16, servo4trim, &beatConfig.servo4.trim)
PARAM_ADD(PARAM_INT8,  servo1sign, &beatConfig.servo1.sign)
PARAM_ADD(PARAM_INT8,  servo2sign, &beatConfig.servo2.sign)
PARAM_ADD(PARAM_INT8,  servo3sign, &beatConfig.servo3.sign)
PARAM_ADD(PARAM_INT8,  servo4sign, &beatConfig.servo4.sign)

PARAM_ADD(PARAM_FLOAT, bldc1trim, &beatConfig.bldc1.trim)
PARAM_ADD(PARAM_FLOAT, bldc2trim, &beatConfig.bldc2.trim)

PARAM_GROUP_STOP(powerDist)