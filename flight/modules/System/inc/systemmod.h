/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup SystemModule System Module
 * @{
 *
 * @file       systemmod.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      System module
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
#ifndef SYSTEMMOD_H
#define SYSTEMMOD_H

int32_t SystemModInitialize(void);

#endif // SYSTEMMOD_H



#include <systemstats.h>
// #include <flightstatus.h>
#include <stabilizationdesired.h>
#include <attitudestate.h>
#include <ratedesired.h>

#include <actuatordesired.h>
#include <gyrostate.h>
#include "actuatorcommand.h"//syz


void SysLoopOuterloopPrintSyz()
{
    // uint8_t armed;
    // FlightStatusArmedGet(&armed);
    // if (armed == FLIGHTSTATUS_ARMED_ARMED)
    // {

        SystemStatsData stats;
        // Get stats and update
        SystemStatsGet(&stats);
        // uint32_t ft = xTaskGetTickCount() * portTICK_RATE_MS;
        // DEBUG_PRINTF(3, ",%d\t,%d\t", stats.FlightTime, ft);
        DEBUG_PRINTF(3, "\r\nSO,");
        DEBUG_PRINTF(3, "\t%d,", stats.FlightTime);

        StabilizationDesiredData stabilizationDesired;
        StabilizationDesiredGet(&stabilizationDesired);
        // float *stabilizationDesiredAxis = &stabilizationDesired.Roll;

        AttitudeStateData attitudeState;
        AttitudeStateGet(&attitudeState);

        RateDesiredData rateDesired;
        RateDesiredGet(&rateDesired);
        // float *rateDesiredAxis = &rateDesired.Roll;
        // float *rate = &rateDesired.Roll;



        // DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
        //             (int32_t)(stabilizationDesiredAxis[0]*1000),
        //             (int32_t)(stabilizationDesiredAxis[1]*1000),
        //             (int32_t)(stabilizationDesiredAxis[2]*1000));// 外环期望
        DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
                    (int32_t)(stabilizationDesired.Roll*1000),
                    (int32_t)(stabilizationDesired.Pitch*1000),
                    (int32_t)(stabilizationDesired.Yaw*1000));// 外环期望

        DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
                    (int32_t)(attitudeState.Roll*1000),
                    (int32_t)(attitudeState.Pitch*1000),
                    (int32_t)(attitudeState.Yaw*1000));// 外环测量

        // DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
        //             (int32_t)(rateDesiredAxis[0]*1000),
        //             (int32_t)(rateDesiredAxis[1]*1000),
        //             (int32_t)(rateDesiredAxis[2]*1000));// 外环结果 同内环期望，但注意有可能内环AXISLOCK模式会预处理
        DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
                    (int32_t)(rateDesired.Roll*1000),
                    (int32_t)(rateDesired.Pitch*1000),
                    (int32_t)(rateDesired.Yaw*1000));// 外环结果 同内环期望，但注意有可能内环AXISLOCK模式会预处理
    // }

    
}

void SysLoopInnerloopPrintSyz()
{
        // SystemStatsData stats;
        // SystemStatsGet(&stats);
        // DEBUG_PRINTF(3, "\r\nSI,");
        // DEBUG_PRINTF(3, "\t%d,", stats.FlightTime);

        GyroStateData gyroState;
        GyroStateGet(&gyroState);

        ActuatorDesiredData actuator;
        ActuatorDesiredGet(&actuator);

        DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
                    (int32_t)(gyroState.x*1000),
                    (int32_t)(gyroState.y*1000),
                    (int32_t)(gyroState.z*1000));// 内环直接测量，无滤波，未经加工
        
        DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
                    (int32_t)(actuator.Roll*1000),
                    (int32_t)(actuator.Pitch*1000),
                    (int32_t)(actuator.Yaw*1000));// 内环结果

}

void SysLoopActuatorPrintSyz()
{
    ActuatorCommandData command;
    ActuatorCommandGet(&command);

    DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,\t%d,", 
                (int32_t)(command.Channel[2]),
                (int32_t)(command.Channel[3]),
                (int32_t)(command.Channel[4]),
                (int32_t)(command.Channel[5]));// 混控矩阵结果

}