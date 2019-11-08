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

#include "platform.h"
#include "fc/rc.h"
#include "scheduler/scheduler.h"
#include "rx/rx.h"

static FAST_CODE void GYRO_FILTER_FUNCTION_NAME(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_RAW, axis, gyro.rawSensorDev->gyroADCRaw[axis]);
        // scale gyro output to degrees per second
        float gyroADCf = gyro.gyroADC[axis];
        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_SCALED, axis, lrintf(gyroADCf));

#ifdef USE_GYRO_DATA_ANALYSE
        if (isDynamicFilterActive()) {
            if (axis == gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT_FREQ, 3, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_DYN_LPF, 0, lrintf(gyroADCf));
            }
        }
#endif

#ifdef USE_RPM_FILTER
        gyroADCf = rpmFilterGyro(axis, gyroADCf);
#endif


        // apply static notch filters and software lowpass filters
        gyroADCf = gyro.notchFilter1ApplyFn((filter_t *)&gyro.notchFilter1[axis], gyroADCf);
        gyroADCf = gyro.notchFilter2ApplyFn((filter_t *)&gyro.notchFilter2[axis], gyroADCf);
        gyroADCf = gyro.lowpassFilterApplyFn((filter_t *)&gyro.lowpassFilter[axis], gyroADCf);
        gyroADCf = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[axis], gyroADCf);

#ifdef USE_GYRO_DATA_ANALYSE
        if (isDynamicFilterActive()) {
            if (axis == gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT_FREQ, 2, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_DYN_LPF, 3, lrintf(gyroADCf));
            }
            gyroDataAnalysePush(&gyro.gyroAnalyseState, axis, gyroADCf);
            gyroADCf = gyro.notchFilterDynApplyFn((filter_t *)&gyro.notchFilterDyn[axis], gyroADCf);
            gyroADCf = gyro.notchFilterDynApplyFn2((filter_t *)&gyro.notchFilterDyn2[axis], gyroADCf);
        }
#endif

        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_FILTERED, axis, lrintf(gyroADCf));

        gyro.gyroADCf[axis] = gyroADCf;
    }

    //Update Dyn LPF at 100Hz
    if(UseDynBiquad) {
        #define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/
        float MinFreq = gyroConfig()->gyro_lowpass_hz;        
        MinFreq += ((float)(rcData[THROTTLE] - 1000) * 0.1f); //Add 0 - 50Hz

            //Update X
            {
                int axis = X;
                float setPoint      = getSetpointRate(axis);
                float FilterGyro    = gyro.gyroADCf[axis];
                float lpfHz = constrainf( MinFreq + ABS(setPoint - FilterGyro) + ABS(FilterGyro / 4.0f), MinFreq, 500.0f);
                biquadFilterUpdate(&gyro.lowpassFilter[axis].biquadFilterState, lpfHz, gyro.targetLooptime, BIQUAD_Q, FILTER_LPF);
                DEBUG_SET(DEBUG_ALTITUDE, 0, lpfHz);
            }

            //Update Y
            {
                int axis = Y;
                float setPoint      = getSetpointRate(axis);
                float FilterGyro    = gyro.gyroADCf[axis];
                float lpfHz = constrainf( MinFreq + ABS(setPoint - FilterGyro) + ABS(FilterGyro / 4.0f), MinFreq, 500.0f);
                biquadFilterUpdate(&gyro.lowpassFilter[axis].biquadFilterState, lpfHz, gyro.targetLooptime, BIQUAD_Q, FILTER_LPF);
                DEBUG_SET(DEBUG_ALTITUDE, 1, lpfHz);
            }

            //Update Z
            {
                int axis = Z;
                float setPoint      = getSetpointRate(axis);
                float FilterGyro    = gyro.gyroADCf[axis];
                float lpfHz = constrainf( MinFreq + ABS(setPoint - FilterGyro) + ABS(FilterGyro / 4.0f), MinFreq, 500.0f);
                biquadFilterUpdate(&gyro.lowpassFilter[axis].biquadFilterState, lpfHz, gyro.targetLooptime, BIQUAD_Q, FILTER_LPF);
                DEBUG_SET(DEBUG_ALTITUDE, 2, lpfHz);
            }  

            //Save CPU load
            DEBUG_SET(DEBUG_ALTITUDE, 3, MinFreq);
    }

}
