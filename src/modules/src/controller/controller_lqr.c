

#include "controller_lqr.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

static float cmd_servo_left;
static float cmd_servo_right;
static float cmd_thrust_left;
static float cmd_thrust_right;

// the row vectors of the gain matrix K
// this should be copied from Drake
#define LQR_ERROR_VECTOR_LENGTH 12
const static float k1[] = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.11, 0.12};
const static float k2[] = {1.01, 1.02, 1.03, 1.04, 1.05, 1.06, 1.07, 1.08, 1.09, 1.10, 1.11, 1.12};
const static float k3[] = {2.01, 2.02, 2.03, 2.04, 2.05, 2.06, 2.07, 2.08, 2.09, 2.10, 2.11, 2.12};
const static float k4[] = {3.01, 3.02, 3.03, 3.04, 3.05, 3.06, 3.07, 3.08, 3.09, 3.10, 3.11, 3.12};

void controllerLQRInit(void)
{
}

bool controllerLQRTest(void)
{
  return true;
}


void controllerLQR(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
    control->controlMode = controlModeLQR;

    // position, orientation (rpy), velocity, angular velocity
    float errors[LQR_ERROR_VECTOR_LENGTH];
    errors[0]  = state->position.x;
    errors[1]  = state->position.y;
    errors[2]  = state->position.z;
    errors[3]  = state->attitude.roll;
    errors[4]  = state->attitude.pitch;
    errors[5]  = state->attitude.yaw;
    errors[6]  = state->velocity.x;
    errors[7]  = state->velocity.y;
    errors[8]  = state->velocity.z;
    errors[9]  = radians(sensors->gyro.x);
    errors[10] = radians(sensors->gyro.y);
    errors[11] = radians(sensors->gyro.z);

    control->servoLeft = 0.0;
    control->servoRight = 0.0;
    control->thrustLeft = 0.0;
    control->thrustRight = 0.0;
    
    // do matrix multiplication manually
    for (int i = 0; i < LQR_ERROR_VECTOR_LENGTH; i++) {
        control->servoLeft   += k1[i]*errors[i];
        control->servoRight  += k2[i]*errors[i];
        control->thrustLeft  += k3[i]*errors[i];
        control->thrustRight += k4[i]*errors[i];
    }

    // logging
    cmd_servo_left = control->servoLeft;
    cmd_servo_right = control->servoRight;
    cmd_thrust_left = control->thrustLeft;
    cmd_thrust_right = control->thrustRight;
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust_left, &cmd_thrust_left)
LOG_ADD(LOG_FLOAT, cmd_thrust_right, &cmd_thrust_right)
LOG_ADD(LOG_FLOAT, cmd_servo_left, &cmd_thrust_left)
LOG_ADD(LOG_FLOAT, cmd_servo_right, &cmd_thrust_right)
LOG_GROUP_STOP(controller)
