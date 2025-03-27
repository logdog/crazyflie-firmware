#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLQR_s {
       // rows of the K matrix
    // u = -K * x
    float k1[12];
    float k2[12];
    float k3[12];
    float k4[12];

    // Logging variables
    struct vec rpy;
    struct vec rpy_des;
    struct mat33 R_des;
    struct vec omega;
    struct vec omega_r;
    struct vec u;
} controllerLQR_t;


void controllerLQRInit(controllerLQR_t* self);
void controllerLQRReset(controllerLQR_t* self);
void controllerLQR(controllerLQR_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

void controllerLQRFirmwareInit(void);
bool controllerLQRFirmwareTest(void);
void controllerLQRFirmware(control_t *control, const setpoint_t *setpoint,
                                        const sensorData_t *sensors,
                                        const state_t *state,
                                        const uint32_t tick);

#endif //__CONTROLLER_LQR_H__
