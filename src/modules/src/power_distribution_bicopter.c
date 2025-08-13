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
 * @file power_distribution_bicopter.c
 * @author Logan Dihel
 * @brief Tells the motors what to do given the thrust and torque controller outputs
 * @details This file was modified from power_distribution_quadrotor.c
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

// config struct which is customized to the
// version of the bicopter, propeller type, etc.
struct bicopterConfig_s {
    float pwmToThrustA;
    float pwmToThrustB;
    float leftMotorTrim;
    float rightMotorTrim;
    int16_t leftServoTrim;
    int16_t rightServoTrim;
    float usPerDeg;
};

struct bicopterConfig_s bicopterConfig = {

#if defined(CONFIG_PROPELLER_WINDANCER) && defined(CONFIG_BATTERY_550)
    .pwmToThrustA = 0.04415f,
    .pwmToThrustB = 0.04359f,
#elif defined(CONFIG_PROPELLER_HURRICANE) && defined(CONFIG_BATTERY_550)
    .pwmToThrustA = 0.06261942f,
    .pwmToThrustB = 0.22547572f,
#elif defined(CONFIG_PROPELLER_HURRICANE) && defined(CONFIG_BATTERY_1550)
    .pwmToThrustA = 0.05163731f,
    .pwmToThrustB = 0.32107592f,
#else
#error "Unsupported propeller and battery configuration"
#endif

#if defined(CONFIG_BICOPTER_NAME_MELONCOPTER)
//   .leftServoTrim = 32 + 7,
//   .rightServoTrim = 6 + 7,
    .leftServoTrim = 85,
    .rightServoTrim = 60,
#elif defined(CONFIG_BICOPTER_NAME_REDCOPTER)
    .leftServoTrim = 21,
    .rightServoTrim = 6,
#else
#error "MELONCOPTER or REDCOPTER must be selected"
#endif

    .usPerDeg = 11.11f,
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
static float maxServoAngle = 30.0f;

int powerDistributionMotorType(uint32_t id)
{
    return (id == MOTOR_M1 || id == MOTOR_M4);
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
    if (id == MOTOR_M1 || id == MOTOR_M4) {
        return 0;
    }
    else if (id == MOTOR_M2) {
        return 1500 + bicopterConfig.leftServoTrim + 0;
    }
    
    // id == MOTOR_M3
    return 1500 - bicopterConfig.rightServoTrim - 0;
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

static int32_t leftServoDegToMicroseconds(float deg) {
    return 1500 + bicopterConfig.leftServoTrim + bicopterConfig.usPerDeg * deg;
}

static int32_t rightServoDegToMicroseconds(float deg) {
    return 1500 - bicopterConfig.rightServoTrim - bicopterConfig.usPerDeg * deg;
}

static int32_t motorThrustToDSHOT(float motorThrust_N) {
    // given the desired force (N), get the DSHOT value to send to the motors.
    // motorThrustUncapped->motors.m1 is in range [0, UINT16_MAX] which is sent as a DSHOT value

    // Force (N) = pwmToThrustA * Veff^2 + pwmToThrustB * Veff
    float y = (-bicopterConfig.pwmToThrustB + sqrtf(bicopterConfig.pwmToThrustB * bicopterConfig.pwmToThrustB + 4.0f * bicopterConfig.pwmToThrustA * motorThrust_N)) / (2.0f * bicopterConfig.pwmToThrustA);
    
    #ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED
    float vBatt = pmGetBatteryVoltage();
    #else
    float vBatt = 14.8f; // 4S battery nominal voltage
    #endif

    float pwm = y / vBatt;

    // maximum pwm value is 1.0
    // pwmAdjust is a parameter we can set to scale up or down all thrusts
    float m_pwm = fmin(pwm, 1.0f);

    return m_pwm * UINT16_MAX;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
    // powerDistributionGetMaxThrust()
    // int32_t leftMotorThrust = control->thrust / 2 + control->roll;
    // int32_t rightMotorThrust = control->thrust / 2 + control->pitch;
    // float leftServoAngleDeg = (control->roll - control->yaw) / 6000.0f;
    // float rightServoAngleDeg = (control->roll + control->yaw) / 6000.0f;

    // #if defined(CONFIG_BICOPTER_NAME_MELONCOPTER)
    // motorThrustUncapped->motors.m4 = control->thrust * UINT16_MAX * bicopterConfig.leftMotorTrim; // left
    // motorThrustUncapped->motors.m1 = control->thrust * UINT16_MAX * bicopterConfig.rightMotorTrim; // right
    // #elif defined(CONFIG_BICOPTER_NAME_REDCOPTER)
    // motorThrustUncapped->motors.m1 = control->thrust * UINT16_MAX * bicopterConfig.leftMotorTrim; // left
    // motorThrustUncapped->motors.m4 = control->thrust * UINT16_MAX * bicopterConfig.rightMotorTrim; // right
    // #endif

    // motorThrustUncapped->motors.m2 = leftServoDegToMicroseconds(leftServoAngleDeg);
    // motorThrustUncapped->motors.m3 = rightServoDegToMicroseconds(rightServoAngleDeg);
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    // Not implemented yet
}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    // Not implemented yet
}

static void powerDistributionWrench(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    // Not implemented yet
}

static void powerDistributionLQR(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    // get the desired force to be produced by each motor
    #if defined(CONFIG_BICOPTER_NAME_MELONCOPTER)
    float m1_force = control->motorRight_N * bicopterConfig.rightMotorTrim;
    float m4_force = control->motorLeft_N * bicopterConfig.leftMotorTrim;
    #elif defined(CONFIG_BICOPTER_NAME_REDCOPTER)
    float m1_force = control->motorLeft_N * bicopterConfig.leftMotorTrim;
    float m4_force = control->motorRight_N * bicopterConfig.rightMotorTrim;
    #endif
    
    motorThrustUncapped->motors.m1 = motorThrustToDSHOT(m1_force); // left motor
    motorThrustUncapped->motors.m4 = motorThrustToDSHOT(m4_force); // right motor

    // left and right servos
    motorThrustUncapped->motors.m2 = leftServoDegToMicroseconds(control->servoLeft_deg);
    motorThrustUncapped->motors.m3 = rightServoDegToMicroseconds(control->servoRight_deg);
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control, motorThrustUncapped);
        break;
    case controlModeForceTorque:
        powerDistributionForceTorque(control, motorThrustUncapped);
        break;
    case controlModeForce:
        powerDistributionForce(control, motorThrustUncapped);
        break;
    case controlModeWrench:
        powerDistributionWrench(control, motorThrustUncapped);
        break;
    case controlModeLQR:
        powerDistributionLQR(control, motorThrustUncapped);
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
    
    // Servos M2 and M3 (limit servo range)
    motorPwm->motors.m2 = limitThrust(motorThrustBatCompUncapped->motors.m2, leftServoDegToMicroseconds(-maxServoAngle), leftServoDegToMicroseconds(maxServoAngle), &isCapped);
    motorPwm->motors.m3 = limitThrust(motorThrustBatCompUncapped->motors.m3, rightServoDegToMicroseconds(maxServoAngle), rightServoDegToMicroseconds(-maxServoAngle), &isCapped);

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
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, maxThrust, &maxThrust)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, leftMotorTrim, &bicopterConfig.leftMotorTrim)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rightMotorTrim, &bicopterConfig.rightMotorTrim)
PARAM_ADD(PARAM_INT16 | PARAM_PERSISTENT, leftServoTrim, &bicopterConfig.leftServoTrim)
PARAM_ADD(PARAM_INT16 | PARAM_PERSISTENT, rightServoTrim, &bicopterConfig.rightServoTrim)
PARAM_GROUP_STOP(powerDist)