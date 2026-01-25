
#ifndef __CONTROLLER_GEMUS_H__
#define __CONTROLLER_GEMUS_H__

#include "stabilizer_types.h"

void controllerGemusInit(void);
bool controllerGemusTest(void);
void controllerGemus(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_GEMUS_H__
