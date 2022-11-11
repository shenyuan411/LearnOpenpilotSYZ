#include <stdio.h>
#include "pios.h"
#include "pios_debug.h"
#include "pios_board.h"
// #include <systemstats.h>
// // #include <flightstatus.h>
// #include <stabilizationdesired.h>
// #include <attitudestate.h>
// #include <ratedesired.h>

// #include <actuatordesired.h>
// #include <gyrostate.h>
// #include "actuatorcommand.h"//syz
// #if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
// extern uint32_t pios_com_debug_id;
// #define PIOS_COM_DEBUG     (pios_com_debug_id)
// #endif

// //一个单精度浮点数据的发送
// void ANO_SendFloat(int channel, float f_dat)
// {
//     uint8_t tbuf[6];
//     int i;
//     unsigned char *p;

//     for(i = 0; i <= 7; i++)
//         tbuf[i] = 0;

//     p=(unsigned char *)&f_dat;		
//     tbuf[0] = 0x5a;
//     // tbuf[1] = channel;  //0xA1
//     // tbuf[2] = 4;
//     tbuf[1]=(unsigned char)(*(p+3));        //取float类型数据存储在内存中的四个字节
//     tbuf[2]=(unsigned char)(*(p+2));
//     tbuf[3]=(unsigned char)(*(p+1));
//     tbuf[4]=(unsigned char)(*(p+0));

//     for(i=0; i<=4; i++)
//         tbuf[5] += tbuf[i];     //校验和
// //printf("%s",tbuf);		//串口发送字符串
//     PIOS_COM_SendFormattedStringNonBlocking(PIOS_COM_DEBUG, tbuf, 6);
// }



// void SysLoopOuterloopPrintSyz()
// {
//     // uint8_t armed;
//     // FlightStatusArmedGet(&armed);
//     // if (armed == FLIGHTSTATUS_ARMED_ARMED)
//     // {

//         SystemStatsData stats;
//         // Get stats and update
//         SystemStatsGet(&stats);
//         // uint32_t ft = xTaskGetTickCount() * portTICK_RATE_MS;
//         // DEBUG_PRINTF(3, ",%d\t,%d\t", stats.FlightTime, ft);
//         DEBUG_PRINTF(3, "\r\nSO,");
//         DEBUG_PRINTF(3, "\t%d,", stats.FlightTime);

//         StabilizationDesiredData stabilizationDesired;
//         StabilizationDesiredGet(&stabilizationDesired);
//         // float *stabilizationDesiredAxis = &stabilizationDesired.Roll;

//         AttitudeStateData attitudeState;
//         AttitudeStateGet(&attitudeState);

//         RateDesiredData rateDesired;
//         RateDesiredGet(&rateDesired);
//         // float *rateDesiredAxis = &rateDesired.Roll;
//         // float *rate = &rateDesired.Roll;



//         // DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
//         //             (int32_t)(stabilizationDesiredAxis[0]*1000),
//         //             (int32_t)(stabilizationDesiredAxis[1]*1000),
//         //             (int32_t)(stabilizationDesiredAxis[2]*1000));// 外环期望
//         DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
//                     (int32_t)(stabilizationDesired.Roll*1000),
//                     (int32_t)(stabilizationDesired.Pitch*1000),
//                     (int32_t)(stabilizationDesired.Yaw*1000));// 外环期望

//         DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
//                     (int32_t)(attitudeState.Roll*1000),
//                     (int32_t)(attitudeState.Pitch*1000),
//                     (int32_t)(attitudeState.Yaw*1000));// 外环测量

//         // DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
//         //             (int32_t)(rateDesiredAxis[0]*1000),
//         //             (int32_t)(rateDesiredAxis[1]*1000),
//         //             (int32_t)(rateDesiredAxis[2]*1000));// 外环结果 同内环期望，但注意有可能内环AXISLOCK模式会预处理
//         DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
//                     (int32_t)(rateDesired.Roll*1000),
//                     (int32_t)(rateDesired.Pitch*1000),
//                     (int32_t)(rateDesired.Yaw*1000));// 外环结果 同内环期望，但注意有可能内环AXISLOCK模式会预处理
//     // }

    
// }

// void SysLoopInnerloopPrintSyz()
// {
//         // SystemStatsData stats;
//         // SystemStatsGet(&stats);
//         // DEBUG_PRINTF(3, "\r\nSI,");
//         // DEBUG_PRINTF(3, "\t%d,", stats.FlightTime);

//         GyroStateData gyroState;
//         GyroStateGet(&gyroState);

//         ActuatorDesiredData actuator;
//         ActuatorDesiredGet(&actuator);

//         DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
//                     (int32_t)(gyroState.x*1000),
//                     (int32_t)(gyroState.y*1000),
//                     (int32_t)(gyroState.z*1000));// 内环直接测量，无滤波，未经加工
        
//         DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,", 
//                     (int32_t)(actuator.Roll*1000),
//                     (int32_t)(actuator.Pitch*1000),
//                     (int32_t)(actuator.Yaw*1000));// 内环结果

// }

// void SysLoopActuatorPrintSyz()
// {
//     ActuatorCommandData command;
//     ActuatorCommandGet(&command);

//     DEBUG_PRINTF(3, "\t%d,\t%d,\t%d,\t%d,", 
//                 (int32_t)(command.Channel[2]),
//                 (int32_t)(command.Channel[3]),
//                 (int32_t)(command.Channel[4]),
//                 (int32_t)(command.Channel[5]));// 混控矩阵结果

// }