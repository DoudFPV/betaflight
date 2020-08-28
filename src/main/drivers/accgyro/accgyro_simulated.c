/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#endif

#ifdef USE_GYRO_SIMULATED

#include "build/build_config.h"

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_simulated.h"


gyroDev_t *SimGyroDev;

uint32_t sim_gyro_value_index = 0;
#include "simulated_gyro_values.h"  //containing gyro tab 

static void SimGyroInit(gyroDev_t *gyro)
{
    SimGyroDev = gyro;
}

STATIC_UNIT_TESTED bool SimGyroRead(gyroDev_t *gyro)
{
    gyro->gyroADCRaw[X] = sim_gyro_value[sim_gyro_value_index];
    gyro->gyroADCRaw[Y] = 0;
    gyro->gyroADCRaw[Z] = 0;

    sim_gyro_value_index++;
    if(sim_gyro_value_index >= sizeof(sim_gyro_value) / sizeof(sim_gyro_value[0])) {
        sim_gyro_value_index = 0;
    }
    return true;
}

bool SimGyroDetect(gyroDev_t *gyro)
{
    gyro->initFn = SimGyroInit;
    gyro->readFn = SimGyroRead;
    gyro->scale = 1.0f; //1.0f / 16.4f;

    return true;
}
#endif // USE_GYRO_SIMULATED
