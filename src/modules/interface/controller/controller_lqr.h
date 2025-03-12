
#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

void controllerLQRInit(void);
bool controllerLQRTest(void);
void controllerLQR(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_LQR_H__
