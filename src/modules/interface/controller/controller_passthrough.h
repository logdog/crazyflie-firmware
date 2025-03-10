
#ifndef __CONTROLLER_PASSTHROUGH_H__
#define __CONTROLLER_PASSTHROUGH_H__

#include "stabilizer_types.h"

void controllerPassthroughInit(void);
bool controllerPassthroughTest(void);
void controllerPassthrough(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_PASSTHROUGH_H__
