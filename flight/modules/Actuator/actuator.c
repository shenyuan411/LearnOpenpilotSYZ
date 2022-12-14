/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup ActuatorModule Actuator Module
 * @brief Compute servo/motor settings based on @ref ActuatorDesired "desired actuator positions" and aircraft type.
 * This is where all the mixing of channels is computed.
 * @{
 *
 * @file       actuator.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Actuator module. Drives the actuators (servos, motors etc).
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

#include "accessorydesired.h"
#include "actuator.h"
#include "actuatorsettings.h"
#include "systemsettings.h"
#include "actuatordesired.h"
#include "actuatorcommand.h"
#include "flightstatus.h"
#include "mixersettings.h"
#include "mixerstatus.h"
#include "cameradesired.h"
#include "manualcontrolcommand.h"
#include "taskinfo.h"
#undef PIOS_INCLUDE_INSTRUMENTATION
#ifdef PIOS_INCLUDE_INSTRUMENTATION
#include <pios_instrumentation.h>
static int8_t counter;
// Counter 0xAC700001 total Actuator body execution time(excluding queue waits etc).
#endif

// Private constants
#define MAX_QUEUE_SIZE                  2

#if defined(PIOS_ACTUATOR_STACK_SIZE)
#define STACK_SIZE_BYTES                PIOS_ACTUATOR_STACK_SIZE
#else
#define STACK_SIZE_BYTES                1312
#endif

#define TASK_PRIORITY                   (tskIDLE_PRIORITY + 4) // device driver
#define FAILSAFE_TIMEOUT_MS             100
#define MAX_MIX_ACTUATORS               ACTUATORCOMMAND_CHANNEL_NUMELEM

#define CAMERA_BOOT_DELAY_MS            7000

#define ACTUATOR_ONESHOT125_CLOCK       2000000
#define ACTUATOR_ONESHOT125_PULSE_SCALE 4
#define ACTUATOR_PWM_CLOCK              1000000
// Private types


// Private variables
static xQueueHandle queue;
static xTaskHandle taskHandle;

static float lastResult[MAX_MIX_ACTUATORS] = { 0 };
static float filterAccumulator[MAX_MIX_ACTUATORS] = { 0 };
static uint8_t pinsMode[MAX_MIX_ACTUATORS];
// used to inform the actuator thread that actuator update rate is changed
static volatile bool actuator_settings_updated;
// used to inform the actuator thread that mixer settings are changed
static volatile bool mixer_settings_updated;

// Private functions
static void actuatorTask(void *parameters);
static int16_t scaleChannel(float value, int16_t max, int16_t min, int16_t neutral);
static void setFailsafe(const ActuatorSettingsData *actuatorSettings, const MixerSettingsData *mixerSettings);
static float MixerCurve(const float throttle, const float *curve, uint8_t elements);
static bool set_channel(uint8_t mixer_channel, uint16_t value, const ActuatorSettingsData *actuatorSettings);
static void actuator_update_rate_if_changed(const ActuatorSettingsData *actuatorSettings, bool force_update);
static void MixerSettingsUpdatedCb(UAVObjEvent *ev);
static void ActuatorSettingsUpdatedCb(UAVObjEvent *ev);
float ProcessMixer(const int index, const float curve1, const float curve2,
                   const MixerSettingsData *mixerSettings, ActuatorDesiredData *desired,
                   const float period);
// void syz_debug_print(float input,int scaleK);//syz
// this structure is equivalent to the UAVObjects for one mixer.
typedef struct {
    uint8_t type;
    int8_t  matrix[5];
} __attribute__((packed)) Mixer_t;

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorStart()
{
    // Start main task
    xTaskCreate(actuatorTask, "Actuator", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_ACTUATOR, taskHandle);
#ifdef PIOS_INCLUDE_WDG
    PIOS_WDG_RegisterFlag(PIOS_WDG_ACTUATOR);
#endif
    return 0;
}

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorInitialize()
{
    // Register for notification of changes to ActuatorSettings
    ActuatorSettingsInitialize();
    ActuatorSettingsConnectCallback(ActuatorSettingsUpdatedCb);

    // Register for notification of changes to MixerSettings
    MixerSettingsInitialize();
    MixerSettingsConnectCallback(MixerSettingsUpdatedCb);

    // Listen for ActuatorDesired updates (Primary input to this module)
    ActuatorDesiredInitialize();
    queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
    ActuatorDesiredConnectQueue(queue);

    // Register AccessoryDesired (Secondary input to this module)
    AccessoryDesiredInitialize();

    // Primary output of this module
    ActuatorCommandInitialize();

#ifdef DIAG_MIXERSTATUS
    // UAVO only used for inspecting the internal status of the mixer during debug
    MixerStatusInitialize();
#endif

    return 0;
}
MODULE_INITCALL(ActuatorInitialize, ActuatorStart);

/**
 * @brief Main Actuator module task
 *
 * Universal matrix based mixer for VTOL, helis and fixed wing.
 * Converts desired roll,pitch,yaw and throttle to servo/ESC outputs.
 *
 * Because of how the Throttle ranges from 0 to 1, the motors should too!
 *
 * Note this code depends on the UAVObjects for the mixers being all being the same
 * and in sequence. If you change the object definition, make sure you check the code!
 *
 * @return -1 if error, 0 if success
 */
static void actuatorTask(__attribute__((unused)) void *parameters)
{
    UAVObjEvent ev;
    portTickType lastSysTime;
    portTickType thisSysTime;
    float dTSeconds;
    uint32_t dTMilliseconds;

    ActuatorCommandData command;
    ActuatorDesiredData desired;
    MixerStatusData mixerStatus;
    FlightStatusData flightStatus;
    SystemSettingsThrustControlOptions thrustType;
    float throttleDesired;
    float collectiveDesired;

#ifdef PIOS_INCLUDE_INSTRUMENTATION
    counter = PIOS_Instrumentation_CreateCounter(0xAC700001);
#endif
    /* Read initial values of ActuatorSettings */
    ActuatorSettingsData actuatorSettings;

    actuator_settings_updated = false;
    ActuatorSettingsGet(&actuatorSettings);

    /* Read initial values of MixerSettings */
    MixerSettingsData mixerSettings;
    mixer_settings_updated = false;
    MixerSettingsGet(&mixerSettings);

    /* Force an initial configuration of the actuator update rates */
    actuator_update_rate_if_changed(&actuatorSettings, true);

    // Go to the neutral (failsafe) values until an ActuatorDesired update is received
    setFailsafe(&actuatorSettings, &mixerSettings);

    // Main task loop
    lastSysTime = xTaskGetTickCount();
    while (1) {
#ifdef PIOS_INCLUDE_WDG
        PIOS_WDG_UpdateFlag(PIOS_WDG_ACTUATOR);
#endif

        // Wait until the ActuatorDesired object is updated
        uint8_t rc = xQueueReceive(queue, &ev, FAILSAFE_TIMEOUT_MS / portTICK_RATE_MS);
#ifdef PIOS_INCLUDE_INSTRUMENTATION
        PIOS_Instrumentation_TimeStart(counter);
#endif
        /* Process settings updated events even in timeout case so we always act on the latest settings */
        if (actuator_settings_updated) {
            actuator_settings_updated = false;
            ActuatorSettingsGet(&actuatorSettings);
            actuator_update_rate_if_changed(&actuatorSettings, false);
        }
        if (mixer_settings_updated) {
            mixer_settings_updated = false;
            MixerSettingsGet(&mixerSettings);
        }

        if (rc != pdTRUE) {
            /* Update of ActuatorDesired timed out.  Go to failsafe */
            setFailsafe(&actuatorSettings, &mixerSettings);
            continue;
        }

        // Check how long since last update
        thisSysTime    = xTaskGetTickCount();
        dTMilliseconds = (thisSysTime == lastSysTime) ? 1 : (thisSysTime - lastSysTime) * portTICK_RATE_MS;
        lastSysTime    = thisSysTime;
        dTSeconds = dTMilliseconds * 0.001f;

        FlightStatusGet(&flightStatus);
        ActuatorDesiredGet(&desired);
        ActuatorCommandGet(&command);
        SystemSettingsThrustControlGet(&thrustType);

        // read in throttle and collective -demultiplex thrust
        switch (thrustType) {
        case SYSTEMSETTINGS_THRUSTCONTROL_THROTTLE://corpter control ???????????????????????????
            throttleDesired = desired.Thrust;
            ManualControlCommandCollectiveGet(&collectiveDesired);
            break;
        case SYSTEMSETTINGS_THRUSTCONTROL_COLLECTIVE:
            ManualControlCommandThrottleGet(&throttleDesired);
            collectiveDesired = desired.Thrust;
            break;
        default:
            ManualControlCommandThrottleGet(&throttleDesired);
            ManualControlCommandCollectiveGet(&collectiveDesired);
        }

        bool armed = flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED;

        // safety settings
        if (!armed) {
            throttleDesired = 0;
        }
        if (throttleDesired <= 0.00f || !armed) {
            // force set all other controls to zero if throttle is cut (previously set in Stabilization)
            if (actuatorSettings.LowThrottleZeroAxis.Roll == ACTUATORSETTINGS_LOWTHROTTLEZEROAXIS_TRUE) {
                desired.Roll = 0;
            }
            if (actuatorSettings.LowThrottleZeroAxis.Pitch == ACTUATORSETTINGS_LOWTHROTTLEZEROAXIS_TRUE) {
                desired.Pitch = 0;
            }
            if (actuatorSettings.LowThrottleZeroAxis.Yaw == ACTUATORSETTINGS_LOWTHROTTLEZEROAXIS_TRUE) {
                desired.Yaw = 0;
            }
        }

#ifdef DIAG_MIXERSTATUS
        MixerStatusGet(&mixerStatus);
#endif
        int nMixers     = 0;
        Mixer_t *mixers = (Mixer_t *)&mixerSettings.Mixer1Type;
        for (int ct = 0; ct < MAX_MIX_ACTUATORS; ct++) {
            if (mixers[ct].type != MIXERSETTINGS_MIXER1TYPE_DISABLED) {
                nMixers++;
            }
        }
        if ((nMixers < 2) && !ActuatorCommandReadOnly()) { // Nothing can fly with less than two mixers.
            setFailsafe(&actuatorSettings, &mixerSettings); // So that channels like PWM buzzer keep working
            continue;
        }

        AlarmsClear(SYSTEMALARMS_ALARM_ACTUATOR);

        bool activeThrottle   = (throttleDesired < 0.00f || throttleDesired > 0.00f);
        bool positiveThrottle = (throttleDesired > 0.00f);
        bool spinWhileArmed   = actuatorSettings.MotorsSpinWhileArmed == ACTUATORSETTINGS_MOTORSSPINWHILEARMED_TRUE;

        float curve1 = MixerCurve(throttleDesired, mixerSettings.ThrottleCurve1, MIXERSETTINGS_THROTTLECURVE1_NUMELEM);

        // The source for the secondary curve is selectable
        float curve2 = 0;
        AccessoryDesiredData accessory;
        switch (mixerSettings.Curve2Source) {
        case MIXERSETTINGS_CURVE2SOURCE_THROTTLE:
            curve2 = MixerCurve(throttleDesired, mixerSettings.ThrottleCurve2, MIXERSETTINGS_THROTTLECURVE2_NUMELEM);
            break;
        case MIXERSETTINGS_CURVE2SOURCE_ROLL:
            curve2 = MixerCurve(desired.Roll, mixerSettings.ThrottleCurve2, MIXERSETTINGS_THROTTLECURVE2_NUMELEM);
            break;
        case MIXERSETTINGS_CURVE2SOURCE_PITCH:
            curve2 = MixerCurve(desired.Pitch, mixerSettings.ThrottleCurve2,
                                MIXERSETTINGS_THROTTLECURVE2_NUMELEM);
            break;
        case MIXERSETTINGS_CURVE2SOURCE_YAW:
            curve2 = MixerCurve(desired.Yaw, mixerSettings.ThrottleCurve2, MIXERSETTINGS_THROTTLECURVE2_NUMELEM);
            break;
        case MIXERSETTINGS_CURVE2SOURCE_COLLECTIVE:
            curve2 = MixerCurve(collectiveDesired, mixerSettings.ThrottleCurve2,
                                MIXERSETTINGS_THROTTLECURVE2_NUMELEM);
            break;
        case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY0:
        case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY1:
        case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY2:
        case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY3:
        case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY4:
        case MIXERSETTINGS_CURVE2SOURCE_ACCESSORY5:
            if (AccessoryDesiredInstGet(mixerSettings.Curve2Source - MIXERSETTINGS_CURVE2SOURCE_ACCESSORY0, &accessory) == 0) {
                curve2 = MixerCurve(accessory.AccessoryVal, mixerSettings.ThrottleCurve2, MIXERSETTINGS_THROTTLECURVE2_NUMELEM);
            } else {
                curve2 = 0;
            }
            break;
        }

        float *status = (float *)&mixerStatus; // access status objects as an array of floats

        for (int ct = 0; ct < MAX_MIX_ACTUATORS; ct++) {
            // During boot all camera actuators should be completely disabled (PWM pulse = 0).
            // command.Channel[i] is reused below as a channel PWM activity flag:
            // 0 - PWM disabled, >0 - PWM set to real mixer value using scaleChannel() later.
            // Setting it to 1 by default means "Rescale this channel and enable PWM on its output".
            command.Channel[ct] = 1;

            if (mixers[ct].type == MIXERSETTINGS_MIXER1TYPE_DISABLED) {
                // Set to minimum if disabled.  This is not the same as saying PWM pulse = 0 us
                status[ct] = -1;
                continue;
            }

            if ((mixers[ct].type == MIXERSETTINGS_MIXER1TYPE_MOTOR) || (mixers[ct].type == MIXERSETTINGS_MIXER1TYPE_REVERSABLEMOTOR) || (mixers[ct].type == MIXERSETTINGS_MIXER1TYPE_SERVO)) {
                status[ct] = ProcessMixer(ct, curve1, curve2, &mixerSettings, &desired, dTSeconds);
            } else {
                status[ct] = -1;
            }

            // Motors have additional protection for when to be on
            if (mixers[ct].type == MIXERSETTINGS_MIXER1TYPE_MOTOR) {
                // If not armed or motors aren't meant to spin all the time
                if (!armed ||
                    (!spinWhileArmed && !positiveThrottle)) {
                    filterAccumulator[ct] = 0;
                    lastResult[ct] = 0;
                    status[ct] = -1; // force min throttle
                }
                // If armed meant to keep spinning,
                else if ((spinWhileArmed && !positiveThrottle) ||
                         (status[ct] < 0)) {
                    status[ct] = 0;
                }
            }

            // Reversable Motors are like Motors but go to neutral instead of minimum
            if (mixers[ct].type == MIXERSETTINGS_MIXER1TYPE_REVERSABLEMOTOR) {
                // If not armed or motor is inactive - no "spinwhilearmed" for this engine type
                if (!armed || !activeThrottle) {
                    filterAccumulator[ct] = 0;
                    lastResult[ct] = 0;
                    status[ct] = 0; // force neutral throttle
                }
            }

            // If an accessory channel is selected for direct bypass mode
            // In this configuration the accessory channel is scaled and mapped
            // directly to output.  Note: THERE IS NO SAFETY CHECK HERE FOR ARMING
            // these also will not be updated in failsafe mode.  I'm not sure what
            // the correct behavior is since it seems domain specific.  I don't love
            // this code
            if ((mixers[ct].type >= MIXERSETTINGS_MIXER1TYPE_ACCESSORY0) &&
                (mixers[ct].type <= MIXERSETTINGS_MIXER1TYPE_ACCESSORY5)) {
                if (AccessoryDesiredInstGet(mixers[ct].type - MIXERSETTINGS_MIXER1TYPE_ACCESSORY0, &accessory) == 0) {
                    status[ct] = accessory.AccessoryVal;
                } else {
                    status[ct] = -1;
                }
            }

            if ((mixers[ct].type >= MIXERSETTINGS_MIXER1TYPE_CAMERAROLLORSERVO1) &&
                (mixers[ct].type <= MIXERSETTINGS_MIXER1TYPE_CAMERAYAW)) {
                CameraDesiredData cameraDesired;
                if (CameraDesiredGet(&cameraDesired) == 0) {
                    switch (mixers[ct].type) {
                    case MIXERSETTINGS_MIXER1TYPE_CAMERAROLLORSERVO1:
                        status[ct] = cameraDesired.RollOrServo1;
                        break;
                    case MIXERSETTINGS_MIXER1TYPE_CAMERAPITCHORSERVO2:
                        status[ct] = cameraDesired.PitchOrServo2;
                        break;
                    case MIXERSETTINGS_MIXER1TYPE_CAMERAYAW:
                        status[ct] = cameraDesired.Yaw;
                        break;
                    default:
                        break;
                    }
                } else {
                    status[ct] = -1;
                }

                // Disable camera actuators for CAMERA_BOOT_DELAY_MS after boot
                if (thisSysTime < (CAMERA_BOOT_DELAY_MS / portTICK_RATE_MS)) {
                    command.Channel[ct] = 0;
                }
            }
        }

        // Set real actuator output values scaling them from mixers. All channels
        // will be set except explicitly disabled (which will have PWM pulse = 0).
        for (int i = 0; i < MAX_MIX_ACTUATORS; i++) {
            if (command.Channel[i]) {
                command.Channel[i] = scaleChannel(status[i],
                                                  actuatorSettings.ChannelMax[i],
                                                  actuatorSettings.ChannelMin[i],
                                                  actuatorSettings.ChannelNeutral[i]);
            }
            
            //syz_debug_print(command.Channel[i],1);//syz
        }
        // DEBUG_PRINTF(3, "\r\nA%d\t");
        // DEBUG_PRINTF(3, "%d\t",command.Channel[0]);
        // DEBUG_PRINTF(3, "%d\t",command.Channel[1]);
        // DEBUG_PRINTF(3, "%d\t",command.Channel[2]);
        // DEBUG_PRINTF(3, "%d\t",command.Channel[3]);

        // Store update time
        command.UpdateTime = dTMilliseconds;
        if (command.UpdateTime > command.MaxUpdateTime) {
            command.MaxUpdateTime = command.UpdateTime;
        }
        // Update output object
        ActuatorCommandSet(&command);
        // Update in case read only (eg. during servo configuration)
        ActuatorCommandGet(&command);

#ifdef DIAG_MIXERSTATUS
        MixerStatusSet(&mixerStatus);
#endif


        // Update servo outputs
        bool success = true;

        for (int n = 0; n < ACTUATORCOMMAND_CHANNEL_NUMELEM; ++n) {
            success &= set_channel(n, command.Channel[n], &actuatorSettings);
        }

        PIOS_Servo_Update();

        if (!success) {
            command.NumFailedUpdates++;
            ActuatorCommandSet(&command);
            AlarmsSet(SYSTEMALARMS_ALARM_ACTUATOR, SYSTEMALARMS_ALARM_CRITICAL);
        }
#ifdef PIOS_INCLUDE_INSTRUMENTATION
        PIOS_Instrumentation_TimeEnd(counter);
#endif
    }
}


/**
 * Process mixing for one actuator
 */
float ProcessMixer(const int index, const float curve1, const float curve2,
                   const MixerSettingsData *mixerSettings, ActuatorDesiredData *desired, const float period)
{
    static float lastFilteredResult[MAX_MIX_ACTUATORS];
    const Mixer_t *mixers = (Mixer_t *)&mixerSettings->Mixer1Type; // pointer to array of mixers in UAVObjects
    const Mixer_t *mixer  = &mixers[index];

    float result = ((((float)mixer->matrix[MIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE1]) * curve1) +
                    (((float)mixer->matrix[MIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE2]) * curve2) +
                    (((float)mixer->matrix[MIXERSETTINGS_MIXER1VECTOR_ROLL]) * desired->Roll) +
                    (((float)mixer->matrix[MIXERSETTINGS_MIXER1VECTOR_PITCH]) * desired->Pitch) +
                    (((float)mixer->matrix[MIXERSETTINGS_MIXER1VECTOR_YAW]) * desired->Yaw)) / 128.0f;

    // note: no feedforward for reversable motors yet for safety reasons
    if (mixer->type == MIXERSETTINGS_MIXER1TYPE_MOTOR) {
        if (result < 0.0f) { // idle throttle
            result = 0.0f;
        }

        // feed forward
        float accumulator = filterAccumulator[index];
        accumulator += (result - lastResult[index]) * mixerSettings->FeedForward;
        lastResult[index] = result;
        result += accumulator;
        if (period > 0.0f) {
            if (accumulator > 0.0f) {
                float invFilter = period / mixerSettings->AccelTime;
                if (invFilter > 1) {
                    invFilter = 1;
                }
                accumulator -= accumulator * invFilter;
            } else {
                float invFilter = period / mixerSettings->DecelTime;
                if (invFilter > 1) {
                    invFilter = 1;
                }
                accumulator -= accumulator * invFilter;
            }
        }
        filterAccumulator[index] = accumulator;
        result += accumulator;

        // acceleration limit
        float dt    = result - lastFilteredResult[index];
        float maxDt = mixerSettings->MaxAccel * period;
        if (dt > maxDt) { // we are accelerating too hard
            result = lastFilteredResult[index] + maxDt;
        }
        lastFilteredResult[index] = result;
    }

    return result;
}


/**
 * Interpolate a throttle curve. Throttle input should be in the range 0 to 1.
 * Output is in the range 0 to 1.
 */
static float MixerCurve(const float throttle, const float *curve, uint8_t elements)
{
    float scale = throttle * (float)(elements - 1);
    int idx1    = scale;

    scale -= (float)idx1; // remainder
    if (curve[0] < -1) {
        return throttle;
    }
    if (idx1 < 0) {
        idx1  = 0; // clamp to lowest entry in table
        scale = 0;
    }
    int idx2 = idx1 + 1;
    if (idx2 >= elements) {
        idx2 = elements - 1; // clamp to highest entry in table
        if (idx1 >= elements) {
            idx1 = elements - 1;
        }
    }
    return curve[idx1] * (1.0f - scale) + curve[idx2] * scale;
}


/**
 * Convert channel from -1/+1 to servo pulse duration in microseconds
 */
static int16_t scaleChannel(float value, int16_t max, int16_t min, int16_t neutral)
{
    int16_t valueScaled;

    // Scale
    if (value >= 0.0f) {
        valueScaled = (int16_t)(value * ((float)(max - neutral))) + neutral;
    } else {
        valueScaled = (int16_t)(value * ((float)(neutral - min))) + neutral;
    }

    if (max > min) {
        if (valueScaled > max) {
            valueScaled = max;
        }
        if (valueScaled < min) {
            valueScaled = min;
        }
    } else {
        if (valueScaled < max) {
            valueScaled = max;
        }
        if (valueScaled > min) {
            valueScaled = min;
        }
    }

    return valueScaled;
}

/**
 * Set actuator output to the neutral values (failsafe)
 */
static void setFailsafe(const ActuatorSettingsData *actuatorSettings, const MixerSettingsData *mixerSettings)
{
    /* grab only the parts that we are going to use */
    int16_t Channel[ACTUATORCOMMAND_CHANNEL_NUMELEM];

    ActuatorCommandChannelGet(Channel);

    const Mixer_t *mixers = (Mixer_t *)&mixerSettings->Mixer1Type; // pointer to array of mixers in UAVObjects

    // Reset ActuatorCommand to safe values
    for (int n = 0; n < ACTUATORCOMMAND_CHANNEL_NUMELEM; ++n) {
        if (mixers[n].type == MIXERSETTINGS_MIXER1TYPE_MOTOR) {
            Channel[n] = actuatorSettings->ChannelMin[n];
        } else if (mixers[n].type == MIXERSETTINGS_MIXER1TYPE_SERVO || mixers[n].type == MIXERSETTINGS_MIXER1TYPE_REVERSABLEMOTOR) {
            Channel[n] = actuatorSettings->ChannelNeutral[n];
        } else {
            Channel[n] = 0;
        }
    }

    // Set alarm
    AlarmsSet(SYSTEMALARMS_ALARM_ACTUATOR, SYSTEMALARMS_ALARM_CRITICAL);

    // Update servo outputs
    for (int n = 0; n < ACTUATORCOMMAND_CHANNEL_NUMELEM; ++n) {
        set_channel(n, Channel[n], actuatorSettings);
    }
    // Send the updated command
    PIOS_Servo_Update();

    // Update output object's parts that we changed
    ActuatorCommandChannelSet(Channel);
}

/**
 * determine buzzer or blink sequence
 **/

typedef enum { BUZZ_BUZZER = 0, BUZZ_ARMING = 1, BUZZ_INFO = 2, BUZZ_MAX = 3 } buzzertype;

static inline bool buzzerState(buzzertype type)
{
    // This is for buzzers that take a PWM input

    static uint32_t tune[BUZZ_MAX] = { 0 };
    static uint32_t tunestate[BUZZ_MAX] = { 0 };


    uint32_t newTune = 0;

    if (type == BUZZ_BUZZER) {
        // Decide what tune to play
        if (AlarmsGet(SYSTEMALARMS_ALARM_BATTERY) > SYSTEMALARMS_ALARM_WARNING) {
            newTune = 0b11110110110000; // pause, short, short, short, long
        } else if (AlarmsGet(SYSTEMALARMS_ALARM_GPS) >= SYSTEMALARMS_ALARM_WARNING) {
            newTune = 0x80000000; // pause, short
        } else {
            newTune = 0;
        }
    } else { // BUZZ_ARMING || BUZZ_INFO
        uint8_t arming;
        FlightStatusArmedGet(&arming);
        // base idle tune
        newTune = 0x80000000; // 0b1000...

        // Merge the error pattern for InfoLed
        if (type == BUZZ_INFO) {
            if (AlarmsGet(SYSTEMALARMS_ALARM_BATTERY) > SYSTEMALARMS_ALARM_WARNING) {
                newTune |= 0b00000000001111111011111110000000;
            } else if (AlarmsGet(SYSTEMALARMS_ALARM_GPS) >= SYSTEMALARMS_ALARM_WARNING) {
                newTune |= 0b00000000000000110110110000000000;
            }
        }
        // fast double blink pattern if armed
        if (arming == FLIGHTSTATUS_ARMED_ARMED) {
            newTune |= 0xA0000000; // 0b101000...
        }
    }

    // Do we need to change tune?
    if (newTune != tune[type]) {
        tune[type] = newTune;
        // resynchronize all tunes on change, so they stay in sync
        for (int i = 0; i < BUZZ_MAX; i++) {
            tunestate[i] = tune[i];
        }
    }

    // Play tune
    bool buzzOn     = false;
    static portTickType lastSysTime = 0;
    portTickType thisSysTime = xTaskGetTickCount();
    portTickType dT = 0;

    // For now, only look at the battery alarm, because functions like AlarmsHasCritical() can block for some time; to be discussed
    if (tune[type]) {
        if (thisSysTime > lastSysTime) {
            dT = thisSysTime - lastSysTime;
        } else {
            lastSysTime = 0; // avoid the case where SysTimeMax-lastSysTime <80
        }

        buzzOn = (tunestate[type] & 1);

        if (dT > 80) {
            // Go to next bit in alarm_seq_state
            for (int i = 0; i < BUZZ_MAX; i++) {
                tunestate[i] >>= 1;
                if (tunestate[i] == 0) { // All done, re-start the tune
                    tunestate[i] = tune[i];
                }
            }
            lastSysTime = thisSysTime;
        }
    }
    return buzzOn;
}


#if defined(ARCH_POSIX) || defined(ARCH_WIN32)
static bool set_channel(uint8_t mixer_channel, uint16_t value, const ActuatorSettingsData *actuatorSettings)
{
    return true;
}
#else
static bool set_channel(uint8_t mixer_channel, uint16_t value, const ActuatorSettingsData *actuatorSettings)
{
    switch (actuatorSettings->ChannelType[mixer_channel]) {
    case ACTUATORSETTINGS_CHANNELTYPE_PWMALARMBUZZER:
        PIOS_Servo_Set(actuatorSettings->ChannelAddr[mixer_channel],
                       buzzerState(BUZZ_BUZZER) ? actuatorSettings->ChannelMax[mixer_channel] : actuatorSettings->ChannelMin[mixer_channel]);
        return true;

    case ACTUATORSETTINGS_CHANNELTYPE_ARMINGLED:
        PIOS_Servo_Set(actuatorSettings->ChannelAddr[mixer_channel],
                       buzzerState(BUZZ_ARMING) ? actuatorSettings->ChannelMax[mixer_channel] : actuatorSettings->ChannelMin[mixer_channel]);
        return true;

    case ACTUATORSETTINGS_CHANNELTYPE_INFOLED:
        PIOS_Servo_Set(actuatorSettings->ChannelAddr[mixer_channel],
                       buzzerState(BUZZ_INFO) ? actuatorSettings->ChannelMax[mixer_channel] : actuatorSettings->ChannelMin[mixer_channel]);
        return true;

    case ACTUATORSETTINGS_CHANNELTYPE_PWM:
    {
        uint8_t mode = pinsMode[actuatorSettings->ChannelAddr[mixer_channel]];
        switch (mode) {
        case ACTUATORSETTINGS_BANKMODE_ONESHOT125:
            // Remap 1000-2000 range to 125-250
            PIOS_Servo_Set(actuatorSettings->ChannelAddr[mixer_channel], value / ACTUATOR_ONESHOT125_PULSE_SCALE);
            break;
        default:
            PIOS_Servo_Set(actuatorSettings->ChannelAddr[mixer_channel], value);
            break;
        }
        return true;
    }

#if defined(PIOS_INCLUDE_I2C_ESC)
    case ACTUATORSETTINGS_CHANNELTYPE_MK:
        return PIOS_SetMKSpeed(actuatorSettings->ChannelAddr[mixer_channel], value);

    case ACTUATORSETTINGS_CHANNELTYPE_ASTEC4:
        return PIOS_SetAstec4Speed(actuatorSettings->ChannelAddr[mixer_channel], value);

#endif
    default:
        return false;
    }

    return false;
}
#endif /* if defined(ARCH_POSIX) || defined(ARCH_WIN32) */

/**
 * @brief Update the servo update rate
 */
static void actuator_update_rate_if_changed(const ActuatorSettingsData *actuatorSettings, bool force_update)
{
    static uint16_t prevBankUpdateFreq[ACTUATORSETTINGS_BANKUPDATEFREQ_NUMELEM];
    static uint8_t prevBankMode[ACTUATORSETTINGS_BANKMODE_NUMELEM];
    bool updateMode = force_update || (memcmp(prevBankMode, actuatorSettings->BankMode, sizeof(prevBankMode)) != 0);
    bool updateFreq = force_update || (memcmp(prevBankUpdateFreq, actuatorSettings->BankUpdateFreq, sizeof(prevBankUpdateFreq)) != 0);

    // check if any setting is changed
    if (updateMode || updateFreq) {
        /* Something has changed, apply the settings to HW */

        uint16_t freq[ACTUATORSETTINGS_BANKUPDATEFREQ_NUMELEM];
        uint32_t clock[ACTUATORSETTINGS_BANKUPDATEFREQ_NUMELEM] = { 0 };
        for (uint8_t i = 0; i < ACTUATORSETTINGS_BANKMODE_NUMELEM; i++) {
            if (force_update || (actuatorSettings->BankMode[i] != prevBankMode[i])) {
                PIOS_Servo_SetBankMode(i,
                                       actuatorSettings->BankMode[i] ==
                                       ACTUATORSETTINGS_BANKMODE_PWM ?
                                       PIOS_SERVO_BANK_MODE_PWM :
                                       PIOS_SERVO_BANK_MODE_SINGLE_PULSE
                                       );
            }
            switch (actuatorSettings->BankMode[i]) {
            case ACTUATORSETTINGS_BANKMODE_ONESHOT125:
                freq[i]  = 100; // Value must be small enough so CCr isn't update until the PIOS_Servo_Update is triggered
                clock[i] = ACTUATOR_ONESHOT125_CLOCK; // Setup an 2MHz timer clock
                break;
            case ACTUATORSETTINGS_BANKMODE_PWMSYNC:
                freq[i]  = 100;
                clock[i] = ACTUATOR_PWM_CLOCK;
                break;
            default: // PWM
                freq[i]  = actuatorSettings->BankUpdateFreq[i];
                clock[i] = ACTUATOR_PWM_CLOCK;
                break;
            }
        }

        memcpy(prevBankMode,
               actuatorSettings->BankMode,
               sizeof(prevBankMode));

        PIOS_Servo_SetHz(freq, clock, ACTUATORSETTINGS_BANKUPDATEFREQ_NUMELEM);

        memcpy(prevBankUpdateFreq,
               actuatorSettings->BankUpdateFreq,
               sizeof(prevBankUpdateFreq));
        // retrieve mode from related bank
        for (uint8_t i = 0; i < MAX_MIX_ACTUATORS; i++) {
            uint8_t bank = PIOS_Servo_GetPinBank(i);
            pinsMode[i] = actuatorSettings->BankMode[bank];
        }
    }
}

static void ActuatorSettingsUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    actuator_settings_updated = true;
}

static void MixerSettingsUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    mixer_settings_updated = true;
}

/**
 * @}
 * @}
 */
// void syz_debug_print(float input,int scaleK)
// {
//     uint32_t output_uint;
//     if (input<0)
//     {
//         output_uint = (-1)*input*scaleK;
//         DEBUG_PRINTF(3, "-%d\t", output_uint); // syz
//     }
//     else{
//         output_uint = input*scaleK;
//         DEBUG_PRINTF(3, "%d\t", output_uint); // syz
//     }
// }