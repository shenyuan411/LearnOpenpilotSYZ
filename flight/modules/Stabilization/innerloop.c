/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup StabilizationModule Stabilization Module
 * @brief Stabilization PID loops in an airframe type independent manner
 * @note This object updates the @ref ActuatorDesired "Actuator Desired" based on the
 * PID loops on the @ref AttitudeDesired "Attitude Desired" and @ref AttitudeState "Attitude State"
 * @{
 *
 * @file       innerloop.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2014.
 * @brief      Attitude stabilization module.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <openpilot.h>
#include <pid.h>
#include <sin_lookup.h>
#include <callbackinfo.h>
#include <ratedesired.h>
#include <actuatordesired.h>
#include <gyrostate.h>
#include <airspeedstate.h>
#include <stabilizationstatus.h>
#include <flightstatus.h>
#include <manualcontrolcommand.h>
#include <stabilizationbank.h>
#include <stabilizationdesired.h>
#include <actuatordesired.h>

#include <stabilization.h>
#include <virtualflybar.h>
#include <cruisecontrol.h>
#include "actuatorcommand.h"//syz
// #include <cmath> //syz
#include <pios_com.h>
// Private constants

#define CALLBACK_PRIORITY CALLBACK_PRIORITY_CRITICAL

#define UPDATE_EXPECTED   (1.0f / PIOS_SENSOR_RATE)
#define UPDATE_MIN        1.0e-6f
#define UPDATE_MAX        1.0f
#define UPDATE_ALPHA      1.0e-2f

// Private variables
static DelayedCallbackInfo *callbackHandle;
static float gyro_filtered[3] = { 0, 0, 0 };
static float axis_lock_accum[3] = { 0, 0, 0 };
static uint8_t previous_mode[AXES] = { 255, 255, 255, 255 };
static PiOSDeltatimeConfig timeval;
static float speedScaleFactor = 1.0f;
static float t_sweep = 0.000f; // syz
static float theta = 0.000f; // syz
float sweep_output = 0.00f; // syz
// Private functions
static void stabilizationInnerloopTask();
static void GyroStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev);
#ifdef REVOLUTION
static void AirSpeedUpdatedCb(__attribute__((unused)) UAVObjEvent *ev);
#endif
void syz_debug_print(float input,int scaleK);//syz
float sweep(int T_rec,float dT, float A, float omega_min, float omega_max);// syz

void stabilizationInnerloopInit()
{
    RateDesiredInitialize();
    ActuatorDesiredInitialize();
    GyroStateInitialize();
    StabilizationStatusInitialize();
    FlightStatusInitialize();
    ManualControlCommandInitialize();
    StabilizationDesiredInitialize();
    ActuatorDesiredInitialize();
#ifdef REVOLUTION
    AirspeedStateInitialize();
    AirspeedStateConnectCallback(AirSpeedUpdatedCb);
#endif
    PIOS_DELTATIME_Init(&timeval, UPDATE_EXPECTED, UPDATE_MIN, UPDATE_MAX, UPDATE_ALPHA);

    callbackHandle = PIOS_CALLBACKSCHEDULER_Create(&stabilizationInnerloopTask, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_STABILIZATION1, STACK_SIZE_BYTES);
    GyroStateConnectCallback(GyroStateUpdatedCb);

    // schedule dead calls every FAILSAFE_TIMEOUT_MS to have the watchdog cleared
    PIOS_CALLBACKSCHEDULER_Schedule(callbackHandle, FAILSAFE_TIMEOUT_MS, CALLBACK_UPDATEMODE_LATER);
}

static float get_pid_scale_source_value()
{
    float value;

    switch (stabSettings.stabBank.ThrustPIDScaleSource) {
    case STABILIZATIONBANK_THRUSTPIDSCALESOURCE_MANUALCONTROLTHROTTLE:
        ManualControlCommandThrottleGet(&value);
        break;
    case STABILIZATIONBANK_THRUSTPIDSCALESOURCE_STABILIZATIONDESIREDTHRUST:
        StabilizationDesiredThrustGet(&value);
        break;
    case STABILIZATIONBANK_THRUSTPIDSCALESOURCE_ACTUATORDESIREDTHRUST:
        ActuatorDesiredThrustGet(&value);
        break;
    default:
        ActuatorDesiredThrustGet(&value);
        break;
    }

    if (value < 0) {
        value = 0.0f;
    }

    return value;
}

typedef struct pid_curve_scaler {
    float  x;
    pointf points[5];
} pid_curve_scaler;

static float pid_curve_value(const pid_curve_scaler *scaler)
{
    float y = y_on_curve(scaler->x, scaler->points, sizeof(scaler->points) / sizeof(scaler->points[0]));

    return 1.0f + (IS_REAL(y) ? y : 0.0f);
}

static pid_scaler create_pid_scaler(int axis)
{
    pid_scaler scaler;

    // Always scaled with the this.
    scaler.p = scaler.i = scaler.d = speedScaleFactor;

    if (stabSettings.thrust_pid_scaling_enabled[axis][0]
        || stabSettings.thrust_pid_scaling_enabled[axis][1]
        || stabSettings.thrust_pid_scaling_enabled[axis][2]) {
        const pid_curve_scaler curve_scaler = {
            .x      = get_pid_scale_source_value(),
            .points = {
                { 0.00f, stabSettings.stabBank.ThrustPIDScaleCurve[0] },
                { 0.25f, stabSettings.stabBank.ThrustPIDScaleCurve[1] },
                { 0.50f, stabSettings.stabBank.ThrustPIDScaleCurve[2] },
                { 0.75f, stabSettings.stabBank.ThrustPIDScaleCurve[3] },
                { 1.00f, stabSettings.stabBank.ThrustPIDScaleCurve[4] }
            }
        };

        float curve_value = pid_curve_value(&curve_scaler);

        if (stabSettings.thrust_pid_scaling_enabled[axis][0]) {
            scaler.p *= curve_value;
        }
        if (stabSettings.thrust_pid_scaling_enabled[axis][1]) {
            scaler.i *= curve_value;
        }
        if (stabSettings.thrust_pid_scaling_enabled[axis][2]) {
            scaler.d *= curve_value;
        }
    }

    return scaler;
}

/**
 * WARNING! This callback executes with critical flight control priority every
 * time a gyroscope update happens do NOT put any time consuming calculations
 * in this loop unless they really have to execute with every gyro update
 */
static void stabilizationInnerloopTask()
{
    // watchdog and error handling
    {
#ifdef PIOS_INCLUDE_WDG
        PIOS_WDG_UpdateFlag(PIOS_WDG_STABILIZATION);
#endif
        bool warn  = false;
        bool error = false;
        bool crit  = false;
        // check if outer loop keeps executing
        if (stabSettings.monitor.rateupdates > -64) {
            stabSettings.monitor.rateupdates--;
        }
        if (stabSettings.monitor.rateupdates < -(2 * OUTERLOOP_SKIPCOUNT)) {
            // warning if rate loop skipped more than 2 execution
            warn = true;
        }
        if (stabSettings.monitor.rateupdates < -(4 * OUTERLOOP_SKIPCOUNT)) {
            // critical if rate loop skipped more than 4 executions
            crit = true;
        }
        // check if gyro keeps updating
        if (stabSettings.monitor.gyroupdates < 1) {
            // error if gyro didn't update at all!
            error = true;
        }
        if (stabSettings.monitor.gyroupdates > 1) {
            // warning if we missed a gyro update
            warn = true;
        }
        if (stabSettings.monitor.gyroupdates > 3) {
            // critical if we missed 3 gyro updates
            crit = true;
        }
        stabSettings.monitor.gyroupdates = 0;

        if (crit) {
            AlarmsSet(SYSTEMALARMS_ALARM_STABILIZATION, SYSTEMALARMS_ALARM_CRITICAL);
        } else if (error) {
            AlarmsSet(SYSTEMALARMS_ALARM_STABILIZATION, SYSTEMALARMS_ALARM_ERROR);
        } else if (warn) {
            AlarmsSet(SYSTEMALARMS_ALARM_STABILIZATION, SYSTEMALARMS_ALARM_WARNING);
        } else {
            AlarmsClear(SYSTEMALARMS_ALARM_STABILIZATION);
        }
    }


    RateDesiredData rateDesired;
    ActuatorDesiredData actuator;
    StabilizationStatusInnerLoopData enabled;
    FlightStatusControlChainData cchain;

    RateDesiredGet(&rateDesired);
    ActuatorDesiredGet(&actuator);
    StabilizationStatusInnerLoopGet(&enabled);
    FlightStatusControlChainGet(&cchain);
    float *rate = &rateDesired.Roll;
    float *actuatorDesiredAxis = &actuator.Roll;
    int t;
    float dT;
    dT = PIOS_DELTATIME_GetAverageSeconds(&timeval);

    for (t = 0; t < AXES; t++) {
        bool reinit = (StabilizationStatusInnerLoopToArray(enabled)[t] != previous_mode[t]);
        previous_mode[t] = StabilizationStatusInnerLoopToArray(enabled)[t];

        if (t < STABILIZATIONSTATUS_INNERLOOP_THRUST) {//??????????????????3
            if (reinit) {
                stabSettings.innerPids[t].iAccumulator = 0;
            }
            switch (StabilizationStatusInnerLoopToArray(enabled)[t]) {
            case STABILIZATIONSTATUS_INNERLOOP_VIRTUALFLYBAR:
                stabilization_virtual_flybar(gyro_filtered[t], rate[t], &actuatorDesiredAxis[t], dT, reinit, t, &stabSettings.settings);
                break;
            case STABILIZATIONSTATUS_INNERLOOP_AXISLOCK:
                if (fabsf(rate[t]) > stabSettings.settings.MaxAxisLockRate) {
                    // While getting strong commands act like rate mode
                    axis_lock_accum[t] = 0;
                } else {
                    // For weaker commands or no command simply attitude lock (almost) on no gyro change
                    axis_lock_accum[t] += (rate[t] - gyro_filtered[t]) * dT;
                    axis_lock_accum[t]  = boundf(axis_lock_accum[t], -stabSettings.settings.MaxAxisLock, stabSettings.settings.MaxAxisLock);
                    rate[t] = axis_lock_accum[t] * stabSettings.settings.AxisLockKp;
                }
            // IMPORTANT: deliberately no "break;" here, execution continues with regular RATE control loop to avoid code duplication!
            // keep order as it is, RATE must follow!
            case STABILIZATIONSTATUS_INNERLOOP_RATE:
                // limit rate to maximum configured limits (once here instead of 5 times in outer loop)
                rate[t] = boundf(rate[t],
                                 -StabilizationBankMaximumRateToArray(stabSettings.stabBank.MaximumRate)[t],
                                 StabilizationBankMaximumRateToArray(stabSettings.stabBank.MaximumRate)[t]
                                 );
                pid_scaler scaler = create_pid_scaler(t);
                actuatorDesiredAxis[t] = pid_apply_setpoint(&stabSettings.innerPids[t], &scaler, rate[t], gyro_filtered[t], dT);
                break;
            case STABILIZATIONSTATUS_INNERLOOP_ACRO:
            {
                float stickinput[3];
                stickinput[0] = boundf(rate[0] / stabSettings.stabBank.ManualRate.Roll, -1.0f, 1.0f);
                stickinput[1] = boundf(rate[1] / stabSettings.stabBank.ManualRate.Pitch, -1.0f, 1.0f);
                stickinput[2] = boundf(rate[2] / stabSettings.stabBank.ManualRate.Yaw, -1.0f, 1.0f);
                rate[t] = boundf(rate[t],
                                 -StabilizationBankMaximumRateToArray(stabSettings.stabBank.MaximumRate)[t],
                                 StabilizationBankMaximumRateToArray(stabSettings.stabBank.MaximumRate)[t]
                                 );
                pid_scaler ascaler = create_pid_scaler(t);
                ascaler.i *= boundf(1.0f - (1.5f * fabsf(stickinput[t])), 0.0f, 1.0f); // this prevents Integral from getting too high while controlled manually
                float arate  = pid_apply_setpoint(&stabSettings.innerPids[t], &ascaler, rate[t], gyro_filtered[t], dT);
                float factor = fabsf(stickinput[t]) * stabSettings.stabBank.AcroInsanityFactor;
                actuatorDesiredAxis[t] = factor * stickinput[t] + (1.0f - factor) * arate;
            }
            break;
            case STABILIZATIONSTATUS_INNERLOOP_DIRECT:
            default:
                actuatorDesiredAxis[t] = rate[t];
                break;
            }
        } else {
            switch (StabilizationStatusInnerLoopToArray(enabled)[t]) {
            case STABILIZATIONSTATUS_INNERLOOP_CRUISECONTROL:
                actuatorDesiredAxis[t] = cruisecontrol_apply_factor(rate[t]);
                break;
            case STABILIZATIONSTATUS_INNERLOOP_DIRECT:
            default:
                actuatorDesiredAxis[t] = rate[t];
                break;
            }
        }

        actuatorDesiredAxis[t] = boundf(actuatorDesiredAxis[t], -1.0f, 1.0f);

    }

    actuator.UpdateTime = dT * 1000;

    if (cchain.Stabilization == FLIGHTSTATUS_CONTROLCHAIN_TRUE) {
        ActuatorDesiredSet(&actuator);
    } else {
        // Force all axes to reinitialize when engaged
        for (t = 0; t < AXES; t++) {
            previous_mode[t] = 255;
        }
    }

    if (stabSettings.stabBank.EnablePiroComp == STABILIZATIONBANK_ENABLEPIROCOMP_TRUE && stabSettings.innerPids[0].iLim > 1e-3f && stabSettings.innerPids[1].iLim > 1e-3f) {
        // attempted piro compensation - rotate pitch and yaw integrals (experimental)
        float angleYaw = DEG2RAD(gyro_filtered[2] * dT);
        float sinYaw   = sinf(angleYaw);
        float cosYaw   = cosf(angleYaw);
        float rollAcc  = stabSettings.innerPids[0].iAccumulator / stabSettings.innerPids[0].iLim;
        float pitchAcc = stabSettings.innerPids[1].iAccumulator / stabSettings.innerPids[1].iLim;
        stabSettings.innerPids[0].iAccumulator = stabSettings.innerPids[0].iLim * (cosYaw * rollAcc + sinYaw * pitchAcc);
        stabSettings.innerPids[1].iAccumulator = stabSettings.innerPids[1].iLim * (cosYaw * pitchAcc - sinYaw * rollAcc);
    }

    {
        uint8_t armed;
        FlightStatusArmedGet(&armed);
        float throttleDesired;
        ManualControlCommandThrottleGet(&throttleDesired);
        if (armed != FLIGHTSTATUS_ARMED_ARMED ||
            ((stabSettings.settings.LowThrottleZeroIntegral == STABILIZATIONSETTINGS_LOWTHROTTLEZEROINTEGRAL_TRUE) && throttleDesired < 0)) {
            // Force all axes to reinitialize when engaged
            for (t = 0; t < AXES; t++) {
                previous_mode[t] = 255;
            }
        }
    }
    
    // int scaleK = 1000;
    uint8_t armed;
    FlightStatusArmedGet(&armed);
    if (armed == FLIGHTSTATUS_ARMED_ARMED)
    {
        // ActuatorCommandData command;
        // ActuatorCommandGet(&command);
        // uint32_t ft = xTaskGetTickCount() * portTICK_RATE_MS;// ?????????????????? ??????ms

        // DEBUG_PRINTF(3, "%d",(int32_t)(rate[2]*1000));// ?????????????????????1000???
        // DEBUG_PRINTF(3, "%d",(int32_t)(gyro_filtered[2]*1000));// ?????????????????????????????????????????????????????????1000???
        // DEBUG_PRINTF(3, "%d",(int32_t)(actuatorDesiredAxis[2]*1000));// ?????????????????????1000???
        // DEBUG_PRINTF(3, "%d",(int16_t)command.Channel[0]);// ???????????????
        // DEBUG_PRINTF(3, "%d",(int16_t)command.Channel[1]);// ???????????????

        // ????????????????????????????????????????????????
        sweep_output = sweep(30,dT,1.0f,3.0f, 52.0f);
        // actuatorDesiredAxis[2] += sweep_output;

        // PIOS_SendFloat48(pios_com_debug_id, rate[2], gyro_filtered[2], actuatorDesiredAxis[2]);
        
        // PIOS_SendUInt32(pios_com_debug_id, xTaskGetTickCount() * portTICK_RATE_MS);// ??????????????????????????????????????????????????????????????????????????????????????????
        PIOS_SendFloat48(pios_com_debug_id, rate[2], gyro_filtered[2], sweep_output);
        // PIOS_SendInt16(pios_com_debug_id, command.Channel[0], command.Channel[1]);
        // ?????????????????????flight/pios/common/pios_com.c
    }else 
    {
        t_sweep = 0.0f; // ???????????????????????????????????????????????????
    }

    

    PIOS_CALLBACKSCHEDULER_Schedule(callbackHandle, FAILSAFE_TIMEOUT_MS, CALLBACK_UPDATEMODE_LATER);
}


static void GyroStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    GyroStateData gyroState;

    GyroStateGet(&gyroState);

    gyro_filtered[0] = gyro_filtered[0] * stabSettings.gyro_alpha + gyroState.x * (1 - stabSettings.gyro_alpha);
    gyro_filtered[1] = gyro_filtered[1] * stabSettings.gyro_alpha + gyroState.y * (1 - stabSettings.gyro_alpha);
    gyro_filtered[2] = gyro_filtered[2] * stabSettings.gyro_alpha + gyroState.z * (1 - stabSettings.gyro_alpha);

    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
    stabSettings.monitor.gyroupdates++;
}

#ifdef REVOLUTION
static void AirSpeedUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    // Scale PID coefficients based on current airspeed estimation - needed for fixed wing planes
    AirspeedStateData airspeedState;

    AirspeedStateGet(&airspeedState);
    if (stabSettings.settings.ScaleToAirspeed < 0.1f || airspeedState.CalibratedAirspeed < 0.1f) {
        // feature has been turned off
        speedScaleFactor = 1.0f;
    } else {
        // scale the factor to be 1.0 at the specified airspeed (for example 10m/s) but scaled by 1/speed^2
        speedScaleFactor = boundf((stabSettings.settings.ScaleToAirspeed * stabSettings.settings.ScaleToAirspeed) / (airspeedState.CalibratedAirspeed * airspeedState.CalibratedAirspeed),
                                  stabSettings.settings.ScaleToAirspeedLimits.Min,
                                  stabSettings.settings.ScaleToAirspeedLimits.Max);
    }
}
#endif

/**
 * @}
 * @}
 */


// void syz_debug_print(float input,int scaleK)
// {
//     int32_t output_int;
//     output_int = input*scaleK;
//     DEBUG_PRINTF(3, "%d", output_int); // syz
// }


/// @brief ???????????????????????????????????????fabian.wang??????????????????????????????????????????????????????????????????????????? P90
/// @param T_rec ???????????????????????????????????????????????????
/// @param dT ????????????????????????????????????0.002s
/// @param A ??????-A???+A
/// @param omega_min ?????????????????????
/// @param omega_max ?????????????????????
/// @return ??????????????????
float sweep(int T_rec,float dT, float A, float omega_min, float omega_max)
{
    const float C1 = 4.0f;
    const float C2 = 0.0187f;

    //?????????????????????????????????????????????
    float K = 0.0f;
    float omega = 0.0f;
    float delta = 0.00f;

    if (t_sweep < T_rec){
        t_sweep += dT;
        K = C2 * (expf(C1*t_sweep/T_rec) - 1);
        omega = omega_min + K*(omega_max -omega_min);
        theta += omega * dT;
        delta = A * sinf(theta);
    }else{
        t_sweep += dT;// ???????????????????????????????????????
        delta = 0.0f;
    }

    return delta;

}

