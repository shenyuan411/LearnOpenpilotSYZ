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
 * @file       outerloop.c
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
#include <callbackinfo.h>
#include <ratedesired.h>
#include <stabilizationdesired.h>
#include <attitudestate.h>
#include <stabilizationstatus.h>
#include <flightstatus.h>
#include <manualcontrolcommand.h>
#include <stabilizationbank.h>

#include <stabilization.h>
#include <cruisecontrol.h>
#include <altitudeloop.h>
#include <CoordinateConversions.h>
#include <stdio.h>
#include <string.h> //syz
#include <ratedesired.h>
#include <actuatordesired.h>
#include <gyrostate.h>
#include "actuatorcommand.h"//syz
// Private constants

#define CALLBACK_PRIORITY CALLBACK_PRIORITY_REGULAR

#define UPDATE_EXPECTED (1.0f / PIOS_SENSOR_RATE)
#define UPDATE_MIN 1.0e-6f
#define UPDATE_MAX 1.0f
#define UPDATE_ALPHA 1.0e-2f

// Private variables
static DelayedCallbackInfo *callbackHandle;
static AttitudeStateData attitude;

static uint8_t previous_mode[AXES] = {255, 255, 255, 255};
static PiOSDeltatimeConfig timeval;

// Private functions
static void stabilizationOuterloopTask();
static void AttitudeStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev);
// void float2u8Arry(uint8_t *u8Arry, float *floatdata, bool key);//syz
//void float2char(float value/*需要转换的值*/,char* cSendBuff/*结果存储的数组*/, int Decimals/*小数位的长度*/);//syz
void syz_debug_print_out(float input,int scaleK);//syz
void stabilizationOuterloopInit()
{
    RateDesiredInitialize();
    StabilizationDesiredInitialize();
    AttitudeStateInitialize();
    StabilizationStatusInitialize();
    FlightStatusInitialize();
    ManualControlCommandInitialize();

    PIOS_DELTATIME_Init(&timeval, UPDATE_EXPECTED, UPDATE_MIN, UPDATE_MAX, UPDATE_ALPHA);

    callbackHandle = PIOS_CALLBACKSCHEDULER_Create(&stabilizationOuterloopTask, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_STABILIZATION0, STACK_SIZE_BYTES);
    AttitudeStateConnectCallback(AttitudeStateUpdatedCb);
}

/**
 * WARNING! This callback executes with critical flight control priority every
 * time a gyroscope update happens do NOT put any time consuming calculations
 * in this loop unless they really have to execute with every gyro update
 */
static void stabilizationOuterloopTask()
{
    AttitudeStateData attitudeState;
    RateDesiredData rateDesired;
    StabilizationDesiredData stabilizationDesired;
    StabilizationStatusOuterLoopData enabled;

    AttitudeStateGet(&attitudeState);
    StabilizationDesiredGet(&stabilizationDesired);
    RateDesiredGet(&rateDesired);
    StabilizationStatusOuterLoopGet(&enabled);
    float *stabilizationDesiredAxis = &stabilizationDesired.Roll;
    float *rateDesiredAxis = &rateDesired.Roll;
    int t;
    float dT = PIOS_DELTATIME_GetAverageSeconds(&timeval);

    float local_error[3];
    // uint32_t attitudeState_uint[3];
    // uint32_t attitudeDesired_uint[3];
    int scaleK = 1000;
    {
#if defined(PIOS_QUATERNION_STABILIZATION)
        // Quaternion calculation of error in each axis.  Uses more memory.
        float rpy_desired[3];
        float q_desired[4];
        float q_error[4];

        for (t = 0; t < 3; t++)
        {
            switch (StabilizationStatusOuterLoopToArray(enabled)[t])
            {
            case STABILIZATIONSTATUS_OUTERLOOP_ATTITUDE:
            case STABILIZATIONSTATUS_OUTERLOOP_RATTITUDE:
            case STABILIZATIONSTATUS_OUTERLOOP_WEAKLEVELING:
                rpy_desired[t] = stabilizationDesiredAxis[t];
                break;
            case STABILIZATIONSTATUS_OUTERLOOP_DIRECT:
            default:
                rpy_desired[t] = ((float *)&attitudeState.Roll)[t];
                break;
            }
        }

        RPY2Quaternion(rpy_desired, q_desired);
        quat_inverse(q_desired);
        quat_mult(q_desired, &attitudeState.q1, q_error);
        quat_inverse(q_error);
        Quaternion2RPY(q_error, local_error);

#else  /* if defined(PIOS_QUATERNION_STABILIZATION) */
        // Simpler algorithm for CC, less memory
        local_error[0] = stabilizationDesiredAxis[0] - attitudeState.Roll;
        local_error[1] = stabilizationDesiredAxis[1] - attitudeState.Pitch;
        local_error[2] = stabilizationDesiredAxis[2] - attitudeState.Yaw;

        // find shortest way
        float modulo = fmodf(local_error[2] + 180.0f, 360.0f);
        if (modulo < 0)
        {
            local_error[2] = modulo + 180.0f;
        }
        else
        {
            local_error[2] = modulo - 180.0f;
        }
#endif /* if defined(PIOS_QUATERNION_STABILIZATION) */
    }
 
    DEBUG_PRINTF(3, "\r\nO\t");
    for (int i = 0; i < 3; i++)
    {
        syz_debug_print_out(stabilizationDesiredAxis[i],scaleK);
    }
    syz_debug_print_out(attitudeState.Roll,scaleK);
    syz_debug_print_out(attitudeState.Pitch,scaleK);
    syz_debug_print_out(attitudeState.Yaw,scaleK);


    
    // memset(strff, 0, sizeof(strff));
    // uint8_t temp2 = attitudeState.Yaw;

    // float_t temp0 = 11370.2003655f;
    // double_t temp1 = temp0;
    // snprintf(strff,11,"%.4f",temp1);
    // DEBUG_PRINTF(3, "pushdouble  %s\r\n", strff); // syz
    // float2char(temp0, strff, 4);
    // DEBUG_PRINTF(3, "push:%s \r\n", strff); // syz??????????????

    // // sprintf(strff,"%.4f", temp2);
    // DEBUG_PRINTF(3, "My local_error[0]roll's value is: %d\r\n", temp2);//syz

    // double_t pi = 7.123145f, test;
    // char byte[4];
    // memcpy(byte, &pi, sizeof(float));
    // DEBUG_PRINTF(3, "%d\r\n", byte[0]);
    // memcpy(&test, byte, sizeof(float));
    // DEBUG_PRINTF(3, "result %f \n", test);

    // uint8_t u8data[4];
    // float2u8Arry(u8data, &temp0, false);
    // DEBUG_PRINTF(3, "u8data  %d \r\n", u8data[0]); // syz

    for (t = 0; t < AXES; t++)
    {
        bool reinit = (StabilizationStatusOuterLoopToArray(enabled)[t] != previous_mode[t]);
        previous_mode[t] = StabilizationStatusOuterLoopToArray(enabled)[t];

        if (t < STABILIZATIONSTATUS_OUTERLOOP_THRUST)
        {
            if (reinit)
            {
                stabSettings.outerPids[t].iAccumulator = 0;
            }
            switch (StabilizationStatusOuterLoopToArray(enabled)[t])
            {
            case STABILIZATIONSTATUS_OUTERLOOP_ATTITUDE:
                rateDesiredAxis[t] = pid_apply(&stabSettings.outerPids[t], local_error[t], dT);
                break;
            case STABILIZATIONSTATUS_OUTERLOOP_RATTITUDE:
            {
                float stickinput[3];
                stickinput[0] = boundf(stabilizationDesiredAxis[0] / stabSettings.stabBank.RollMax, -1.0f, 1.0f);
                stickinput[1] = boundf(stabilizationDesiredAxis[1] / stabSettings.stabBank.PitchMax, -1.0f, 1.0f);
                stickinput[2] = boundf(stabilizationDesiredAxis[2] / stabSettings.stabBank.YawMax, -1.0f, 1.0f);
                float rateDesiredAxisRate = stickinput[t] * StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t];
                // limit corrective rate to maximum rates to not give it overly large impact over manual rate when joined together
                rateDesiredAxis[t] = boundf(pid_apply(&stabSettings.outerPids[t], local_error[t], dT),
                                            -StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t],
                                            StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t]);
                // Compute the weighted average rate desired
                // Using max() rather than sqrt() for cpu speed;
                // - this makes the stick region into a square;
                // - this is a feature!
                // - hold a roll angle and add just pitch without the stick sensitivity changing
                float magnitude = fabsf(stickinput[t]);
                if (t < 2)
                {
                    magnitude = fmaxf(fabsf(stickinput[0]), fabsf(stickinput[1]));
                }

// modify magnitude to move the Att to Rate transition to the place
// specified by the user
// we are looking for where the stick angle == transition angle
// and the Att rate equals the Rate rate
// that's where Rate x (1-StickAngle) [Attitude pulling down max X Ratt proportion]
// == Rate x StickAngle [Rate pulling up according to stick angle]
// * StickAngle [X Ratt proportion]
// so 1-x == x*x or x*x+x-1=0 where xE(0,1)
// (-1+-sqrt(1+4))/2 = (-1+sqrt(5))/2
// and quadratic formula says that is 0.618033989f
// I tested 14.01 and came up with .61 without even remembering this number
// I thought that moving the P,I, and maxangle terms around would change this value
// and that I would have to take these into account, but varying
// all P's and I's by factors of 1/2 to 2 didn't change it noticeably
// and varying maxangle from 4 to 120 didn't either.
// so for now I'm not taking these into account
// while working with this, it occurred to me that Attitude mode,
// set up with maxangle=190 would be similar to Ratt, and it is.
#define STICK_VALUE_AT_MODE_TRANSITION 0.618033989f

                // the following assumes the transition would otherwise be at 0.618033989f
                // and THAT assumes that Att ramps up to max roll rate
                // when a small number of degrees off of where it should be

                // if below the transition angle (still in attitude mode)
                // '<=' instead of '<' keeps rattitude_mode_transition_stick_position==1.0 from causing DZ
                if (magnitude <= stabSettings.rattitude_mode_transition_stick_position)
                {
                    magnitude *= STICK_VALUE_AT_MODE_TRANSITION / stabSettings.rattitude_mode_transition_stick_position;
                }
                else
                {
                    magnitude = (magnitude - stabSettings.rattitude_mode_transition_stick_position) * (1.0f - STICK_VALUE_AT_MODE_TRANSITION) / (1.0f - stabSettings.rattitude_mode_transition_stick_position) + STICK_VALUE_AT_MODE_TRANSITION;
                }
                rateDesiredAxis[t] = (1.0f - magnitude) * rateDesiredAxis[t] + magnitude * rateDesiredAxisRate;
            }
            break;
            case STABILIZATIONSTATUS_OUTERLOOP_WEAKLEVELING:
                // FIXME: local_error[] is rate - attitude for Weak Leveling
                // The only ramifications are:
                // Weak Leveling Kp is off by a factor of 3 to 12 and may need a different default in GCS
                // Changing Rate mode max rate currently requires a change to Kp
                // That would be changed to Attitude mode max angle affecting Kp
                // Also does not take dT into account
                {
                    float stickinput[3];
                    stickinput[0] = boundf(stabilizationDesiredAxis[0] / stabSettings.stabBank.RollMax, -1.0f, 1.0f);
                    stickinput[1] = boundf(stabilizationDesiredAxis[1] / stabSettings.stabBank.PitchMax, -1.0f, 1.0f);
                    stickinput[2] = boundf(stabilizationDesiredAxis[2] / stabSettings.stabBank.YawMax, -1.0f, 1.0f);
                    float rate_input = stickinput[t] * StabilizationBankManualRateToArray(stabSettings.stabBank.ManualRate)[t];
                    float weak_leveling = local_error[t] * stabSettings.settings.WeakLevelingKp;
                    weak_leveling = boundf(weak_leveling, -stabSettings.settings.MaxWeakLevelingRate, stabSettings.settings.MaxWeakLevelingRate);

                    // Compute desired rate as input biased towards leveling
                    rateDesiredAxis[t] = rate_input + weak_leveling;
                }
                break;
            case STABILIZATIONSTATUS_OUTERLOOP_DIRECT:
            default:
                rateDesiredAxis[t] = stabilizationDesiredAxis[t];
                break;
            }
        }
        else
        {
            switch (StabilizationStatusOuterLoopToArray(enabled)[t])
            {
#ifdef REVOLUTION
            case STABILIZATIONSTATUS_OUTERLOOP_ALTITUDE:
                rateDesiredAxis[t] = stabilizationAltitudeHold(stabilizationDesiredAxis[t], ALTITUDEHOLD, reinit);
                break;
            case STABILIZATIONSTATUS_OUTERLOOP_ALTITUDEVARIO:
                rateDesiredAxis[t] = stabilizationAltitudeHold(stabilizationDesiredAxis[t], ALTITUDEVARIO, reinit);
                break;
#endif /* REVOLUTION */
            case STABILIZATIONSTATUS_OUTERLOOP_DIRECT:
            default:
                rateDesiredAxis[t] = stabilizationDesiredAxis[t];
                break;
            }
        }
    }
    syz_debug_print_out(rateDesiredAxis[0],scaleK);
    syz_debug_print_out(rateDesiredAxis[1],scaleK);
    syz_debug_print_out(rateDesiredAxis[2],scaleK);

    RateDesiredSet(&rateDesired);
    {
        uint8_t armed;
        FlightStatusArmedGet(&armed);
        float throttleDesired;
        ManualControlCommandThrottleGet(&throttleDesired);
        if (armed != FLIGHTSTATUS_ARMED_ARMED ||
            ((stabSettings.settings.LowThrottleZeroIntegral == STABILIZATIONSETTINGS_LOWTHROTTLEZEROINTEGRAL_TRUE) && throttleDesired < 0))
        {
            // Force all axes to reinitialize when engaged
            for (t = 0; t < AXES; t++)
            {
                previous_mode[t] = 255;
            }
        }
    }

    // update cruisecontrol based on attitude
    cruisecontrol_compute_factor(&attitudeState, rateDesired.Thrust);
    stabSettings.monitor.rateupdates = 0;

    // syz print
    // RateDesiredData rateDesired;
    ActuatorDesiredData actuator;
    // RateDesiredGet(&rateDesired);
    ActuatorDesiredGet(&actuator);
    // float *rate = &rateDesired.Roll;
    float *actuatorDesiredAxis = &actuator.Roll;
    DEBUG_PRINTF(3, "\r\nIo\t");
    syz_debug_print_out(actuatorDesiredAxis[0],scaleK);
    syz_debug_print_out(actuatorDesiredAxis[1],scaleK);
    syz_debug_print_out(actuatorDesiredAxis[2],scaleK);
    // syz_debug_print_out(gyro_filtered[0],scaleK);
    // syz_debug_print_out(gyro_filtered[1],scaleK);
    // syz_debug_print_out(gyro_filtered[2],scaleK);
    // syz_debug_print_out(rate[0],scaleK);
    // syz_debug_print_out(rate[1],scaleK);
    // syz_debug_print_out(rate[2],scaleK);

    // ActuatorCommandData command;
    // ActuatorCommandGet(&command);
    
    // DEBUG_PRINTF(3, "\r\nA\t");
    // DEBUG_PRINTF(3, "%d\t",command.Channel[2]);
    // DEBUG_PRINTF(3, "%d\t",command.Channel[3]);
    // DEBUG_PRINTF(3, "%d\t",command.Channel[4]);
    // DEBUG_PRINTF(3, "%d\t",command.Channel[5]);
}

static void AttitudeStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    // to reduce CPU utilization, outer loop is not executed on every state update
    static uint8_t cpusaver = 0;

    if ((cpusaver++ % OUTERLOOP_SKIPCOUNT) == 0)
    {
        // this does not need mutex protection as both eventdispatcher and stabi run in same callback task!
        AttitudeStateGet(&attitude);
        PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
    }
}

/**
 * @}
 * @}
 */
// void float2u8Arry(uint8_t *u8Arry, float *floatdata, bool key)
// {
//     uint8_t farray[4];
//     *(float *)farray = *floatdata;
//     if (key == true)
//     {
//         u8Arry[3] = farray[0];
//         u8Arry[2] = farray[1];
//         u8Arry[1] = farray[2];
//         u8Arry[0] = farray[3];
//     }
//     else
//     {
//         u8Arry[0] = farray[0];
//         u8Arry[1] = farray[1];
//         u8Arry[2] = farray[2];
//         u8Arry[3] = farray[3];
//     }
// }
//float转char数组，一次转一个
// void float2char(float value/*需要转换的值*/,
//     char* cSendBuff/*结果存储的数组*/, int Decimals/*小数位的长度*/){
// 	int i = 1, k = 0;
// 	int integer = abs(value);//整数部分
// 	int decimal = (abs(value) - integer)*pow(10, Decimals);//小数部分
// 	int temp = integer;
// 	if (value < 0)cSendBuff[k++] = '-';//如果小于0，加个负号
// 	while (temp /= 10)
// 	{
// 		i*=10;
// 	}
// 	while (integer) {
// 		cSendBuff[k++] = integer / i + '0';
// 		integer %= i;
// 		i /= 10;
// 	}
// 	if (Decimals == 0) {	//如果没有小数位，直接返回
// 		cSendBuff[k++] = '\0';
// 		return;
// 	}
// 	cSendBuff[k++] = '.';    //加小数点
// 	temp = decimal;
// 	i = 1;
// 	while (temp /= 10)
// 	{
// 		i *= 10;
// 	}
// 	while (decimal) {
// 		cSendBuff[k++] = decimal / i + '0';
// 		decimal %= i;
// 		i /= 10;
// 	}
// 	cSendBuff[k++] = '\0';
	
// }

void syz_debug_print_out(float input,int scaleK)
{
    int32_t output_uint;
    output_uint = input*scaleK;
    DEBUG_PRINTF(3, "%d\t", output_uint); // syz
}