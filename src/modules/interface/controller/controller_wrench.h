
#ifndef __CONTROLLER_WRENCH_H__
#define __CONTROLLER_WRENCH_H__

#include "stabilizer_types.h"

void controllerWrenchInit(void);
bool controllerWrenchTest(void);
void controllerWrench(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_WRENCH_H__
