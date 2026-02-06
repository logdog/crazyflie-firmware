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
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"


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

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...
  control->controlMode = controlModeBeat;

  if (tick % 1000 == 0)
    DEBUG_PRINT("hello from OOT %d\n", tick);

  
  t += dt / T;
  while (t >= 1.0f) {
    t -= 1.0f;
  }

  control->phiLeft_deg = calculateSchenatoAngle(t, &baseSchenatoProfile); // t is normalized time
  control->phiRight_deg = calculateSchenatoAngle(t, &baseSchenatoProfile); // t is normalized time

  if (t < baseSchenatoProfile.rho) {
    control->psiLeft_deg = -45.0f;
    control->psiRight_deg = -45.0f;
  }
  else {
    control->psiLeft_deg = 45.0f;
    control->psiRight_deg = 45.0f;
  }

  control->thrustLeft_N = 0.5f;
  control->thrustRight_N = 0.5f;
}
