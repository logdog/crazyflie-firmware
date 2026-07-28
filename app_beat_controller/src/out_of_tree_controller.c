/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2024 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details->
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * out_of_tree_controller.c - App layer application of an out of tree controller.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "OOT_BEAT_CONTROLLER"
#include "debug.h"

#include "physicalConstants.h"
#include "param.h"
#include "math3d.h"

struct schenato_e {
  float A;
  float K;
  float rho;
  float gamma;
};

static struct schenato_e baseSchenatoProfile = {
    .A = 1.0f,
    .K = 29.0f,
    .rho = 0.50f,
    .gamma = 0.0f
};

static float calculateSchenatoAngle(float t, struct schenato_e * s) {
    
    float angle;

    if (0 <= t && t <= s->rho) {
        angle = s->A * (1 + s->K) * (1 - 2.0f*t/s->rho) + s->gamma * s->A;
    }
    else {
        angle = s->A * (1 + s->K) * (2.0f*(t - s->rho)/(1 - s->rho) - 1) + s->gamma * s->A;
    }

    return angle;
}

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// The new controller goes here --------------------------------------------
// Move the includes to the the top of the file if you want to
#include "controller.h"

void controllerOutOfTreeInit() {
  // Initialize your controller data here...
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

// the current normalized time
static float t = 0.0f; // normalized time
static float dt = 1/1000.0f; // update rate
static float T = 5.0f; // flapping period

#define LQR_NUM_STATES 12

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLQR_s {
       // rows of the K matrix
    // u = -K * x
    float k1[LQR_NUM_STATES];
    float k2[LQR_NUM_STATES];
    float k3[LQR_NUM_STATES];
    float k4[LQR_NUM_STATES];
    float k5[LQR_NUM_STATES];
    float k6[LQR_NUM_STATES];


    float mass; // bicopter mass in kg
} controllerLQR_t;

// phi_L, phi_R, psi_L, psi_R (radians), f_L, f_R (N)
static controllerLQR_t K = {
  // .k1 = {-0.05443456f, 0.00000000f, 0.00000000f,
  //        0.00000000f, -0.40082823f, 0.00000000f,
  //        -0.08181116f, 0.00000000f, 0.00000000f,
  //        0.00000000f, -0.05787047f, 0.00000000f},
  // .k2 = {-0.05443456f, 0.00000000f, 0.00000000f,
  //        0.00000000f, -0.40082823f, 0.00000000f,
  //        -0.08181116f, 0.00000000f, 0.00000000f,
  //        0.00000000f, -0.05787047f, 0.00000000f},
  // .k3 = {0.07663962f, 0.00000000f, 0.00000000f,
  //        0.00000000f, 0.28917018f, -0.09659549f,
  //        0.10578335f, 0.00000000f, 0.00000000f,
  //        0.00000000f, 0.03412022f, -0.02837284f},
  // .k4 = {0.07663962f, 0.00000000f, 0.00000000f,
  //        0.00000000f, 0.28917018f, 0.09659549f,
  //        0.10578335f, 0.00000000f, 0.00000000f,
  //        0.00000000f, 0.03412022f, 0.02837284f},
  // .k5 = {0.00000000f, -0.64587570f, 0.69586724f,
  //        3.25380798f, 0.00000000f, 0.00000000f,
  //        0.00000000f, -0.91956898f, 0.94960601f,
  //        0.37062771f, 0.00000000f, 0.00000000f},
  // .k6 = {0.00000000f, 0.64587570f, 0.69586724f,
  //        -3.25380798f, 0.00000000f, 0.00000000f,
  //        0.00000000f, 0.91956898f, 0.94960601f,
  //        -0.37062771f, 0.00000000f, 0.00000000f},


  // redcopter LQR parameters
  .k1 = {0,0,0,0,0,0,0,0,0,0,0,0},
  .k2 = {0,0,0,0,0,0,0,0,0,0,0,0},
  .k3 = {0.09149180f, -0.00669075f, 0.00000000f,
         0.04788407f, 0.57753220f, -0.21134638f, 
         0.13835939f, -0.01048333f, 0.00000000f, 
         0.00727996f, 0.06230944f, -0.04034646f},

  .k4 = {0.09149180f, 0.00669075f, -0.00000000f,
         -0.04788407f, 0.57753220f, 0.21134638f, 
         0.13835939f, 0.01048333f, -0.00000000f, 
         -0.00727996f, 0.06230944f, 0.04034646f},

  .k5 = {0.00000000f, -0.63608557f, 2.18637896f,
         3.89758520f, 0.00000000f, 0.10493207f, 
         0.00000000f, -0.95396899f, 1.34283501f, 
         0.39255041f, 0.00000000f, 0.01327680f},

  .k6 = {-0.00000000f, 0.63608557f, 2.18637896f,
         -3.89758520f, -0.00000000f, -0.10493207f, 
         -0.00000000f, 0.95396899f, 1.34283501f, 
         -0.39255041f, -0.00000000f, -0.01327680f},
  .mass = 0.60000000f
};

struct flappingConfig_s {
    enum flappingMode_s {
      disabled = 0,
      waiting = 1,
      enabled = 2,
    } state;
    float hz;
    float phiAmplitudeDeg;
    float psiAmplitudeDeg;
    uint32_t startTick;
};

struct flappingConfig_s flappingConfig = {
  .state = disabled,
  .hz = 1.0f,
  .phiAmplitudeDeg = 20.0f,
  .psiAmplitudeDeg = 15.0f,
  .startTick = 0
};


void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...
  control->controlMode = controlModeBeat;

  if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    return;
  }

  // DEBUG_PRINT("the controller is running!\n");

  // state
  float x[12] = {
    state->position.x, state->position.y, state->position.z,
    radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw),
    state->velocity.x, state->velocity.y, state->velocity.z,
    radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)
  };

  // desired state
  float xd[12] = {0};
  xd[0] = setpoint->position.x;
  xd[1] = setpoint->position.y;
  xd[2] = setpoint->position.z;


  // implement the LQR control law (with flapping, if enabled)
  // u = -K(x - x_desired) + ue + u_delta[k]
  float phiLeft_deg = 0;
  float phiRight_deg = 0;
  float psiLeft_deg = 0;
  float psiRight_deg = 0;
  float thrustLeft_N = 0;
  float thrustRight_N = 0;

  float tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -K.k1[i] * (x[i] - xd[i]);
  }
  phiLeft_deg = degrees(tmp);

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -K.k2[i] * (x[i] - xd[i]);
  }
  phiRight_deg = degrees(tmp);

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -K.k3[i] * (x[i] - xd[i]);
  }
  psiLeft_deg = degrees(tmp);
  
  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -K.k4[i] * (x[i] - xd[i]);
  }
  psiRight_deg = degrees(tmp);

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -K.k5[i] * (x[i] - xd[i]);
  }
  thrustLeft_N = tmp + 9.81f*K.mass/2.0f;

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -K.k6[i] * (x[i] - xd[i]);
  }
  thrustRight_N = tmp + 9.81f*K.mass/2.0f;


  // add the flapping signals on top
  if (flappingConfig.state == waiting && tick % 1000 == 0) {
    flappingConfig.state = enabled;
    flappingConfig.startTick = tick;
  }

  if (flappingConfig.state == enabled) {
    // how much time as passed since we entered the flapping state
    uint32_t elapsedTime = tick - flappingConfig.startTick;

    float phi_deg = flappingConfig.phiAmplitudeDeg*sinf(2*(float)M_PI*flappingConfig.hz*tick/1000.0f);
    float psi_deg = flappingConfig.psiAmplitudeDeg*sinf(2*(float)M_PI*flappingConfig.hz*tick/1000.0f);

    // ramp up over the first second
    if (elapsedTime < 1000) {
      phi_deg *= elapsedTime/1000.0f;
      psi_deg *= elapsedTime/1000.0f;
    }

    phiLeft_deg += phi_deg;
    phiRight_deg += phi_deg;

    psiLeft_deg += psi_deg;
    psiRight_deg += psi_deg;

    if (elapsedTime >= 10000) {
      flappingConfig.state = disabled;
    }
  }
    

  // update the control struct
  control->phiLeft_deg = phiLeft_deg;
  control->phiRight_deg = phiRight_deg;
  // control->phiLeft_deg = 0;
  // control->phiRight_deg = 0;
  control->psiLeft_deg = psiLeft_deg;
  control->psiRight_deg = psiRight_deg;
  control->thrustLeft_N = thrustLeft_N;
  control->thrustRight_N = thrustRight_N;
}

PARAM_GROUP_START(ctrlBeatLQR)
PARAM_ADD(PARAM_UINT8, flap_state, &flappingConfig.state)
PARAM_ADD(PARAM_FLOAT, flap_hz, &flappingConfig.hz)
PARAM_ADD(PARAM_FLOAT, flap_phi, &flappingConfig.phiAmplitudeDeg)
PARAM_ADD(PARAM_FLOAT, flap_psi, &flappingConfig.psiAmplitudeDeg)
PARAM_GROUP_STOP(ctrlBeatLQR)