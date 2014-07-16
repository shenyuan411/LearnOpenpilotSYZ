/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup StabilizationModule Stabilization Module
 * @brief Relay tuning controller
 * @note This object updates the @ref ActuatorDesired "Actuator Desired" based on the
 * PID loops on the @ref AttitudeDesired "Attitude Desired" and @ref AttitudeState "Attitude State"
 * @{
 *
 * @file       stabilization.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
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

#include "openpilot.h"
#include <pios_struct_helper.h>
#include "stabilization.h"
#include "relaytuning.h"
#include "relaytuningsettings.h"
#include "sin_lookup.h"

/**
 * Apply a step function for the stabilization controller and monitor the
 * result
 *
 * Used to  Replace the rate PID with a relay to measure the critical properties of this axis
 * i.e. period and gain
 */
int stabilization_relay_rate(float error, float *output, int axis, bool reinit)
{
    RelayTuningData relay;

    RelayTuningGet(&relay);

    static portTickType lastHighTime;
    static portTickType lastLowTime;

    static float accum_sin, accum_cos;
    static uint32_t accumulated  = 0;

    const uint16_t DEGLITCH_TIME = 20; // ms
    const float AMPLITUDE_ALPHA  = 0.95f;
    const float PERIOD_ALPHA     = 0.95f;

    portTickType thisTime = xTaskGetTickCount();

    static bool rateRelayRunning[3];

    // This indicates the current estimate of the smoothed error.  So when it is high
    // we are waiting for it to go low.
    static bool high = false;

    // On first run initialize estimates to something reasonable
    if (reinit) {
        rateRelayRunning[axis] = false;
        cast_struct_to_array(relay.Period, relay.Period.Roll)[axis] = 200;
        cast_struct_to_array(relay.Gain, relay.Gain.Roll)[axis]     = 0;

        accum_sin   = 0;
        accum_cos   = 0;
        accumulated = 0;

        // These should get reinitialized anyway
        high = true;
        lastHighTime = thisTime;
        lastLowTime  = thisTime;
        RelayTuningSet(&relay);
    }


    RelayTuningSettingsData relaySettings;
    RelayTuningSettingsGet(&relaySettings);

    // Compute output, simple threshold on error
    *output = high ? relaySettings.Amplitude : -relaySettings.Amplitude;

    /**** The code below here is to estimate the properties of the oscillation ****/

    // Make sure the period can't go below limit
    if (cast_struct_to_array(relay.Period, relay.Period.Roll)[axis] < DEGLITCH_TIME) {
        cast_struct_to_array(relay.Period, relay.Period.Roll)[axis] = DEGLITCH_TIME;
    }

    // Project the error onto a sine and cosine of the same frequency
    // to accumulate the average amplitude
    int32_t dT  = thisTime - lastHighTime;
    float phase = ((float)360 * (float)dT) / cast_struct_to_array(relay.Period, relay.Period.Roll)[axis];
    if (phase >= 360) {
        phase = 0;
    }
    accum_sin += sin_lookup_deg(phase) * error;
    accum_cos += cos_lookup_deg(phase) * error;
    accumulated++;

    // Make sure we've had enough time since last transition then check for a change in the output
    bool time_hysteresis = (high ? (thisTime - lastHighTime) : (thisTime - lastLowTime)) > DEGLITCH_TIME;

    if (!high && time_hysteresis && error > relaySettings.HysteresisThresh) {
        /* POSITIVE CROSSING DETECTED */

        float this_amplitude = 2 * sqrtf(accum_sin * accum_sin + accum_cos * accum_cos) / accumulated;
        float this_gain = this_amplitude / relaySettings.Amplitude;

        accumulated = 0;
        accum_sin   = 0;
        accum_cos   = 0;

        if (rateRelayRunning[axis] == false) {
            rateRelayRunning[axis] = true;
            cast_struct_to_array(relay.Period, relay.Period.Roll)[axis] = 200;
            cast_struct_to_array(relay.Gain, relay.Gain.Roll)[axis] = 0;
        } else {
            // Low pass filter each amplitude and period
            cast_struct_to_array(relay.Gain, relay.Gain.Roll)[axis]     =
                cast_struct_to_array(relay.Gain, relay.Gain.Roll)[axis] *
                AMPLITUDE_ALPHA + this_gain * (1 - AMPLITUDE_ALPHA);
            cast_struct_to_array(relay.Period, relay.Period.Roll)[axis] =
                cast_struct_to_array(relay.Period, relay.Period.Roll)[axis] *
                PERIOD_ALPHA + dT * (1 - PERIOD_ALPHA);
        }
        lastHighTime = thisTime;
        high = true;
        RelayTuningSet(&relay);
    } else if (high && time_hysteresis && error < -relaySettings.HysteresisThresh) {
        /* FALLING CROSSING DETECTED */

        lastLowTime = thisTime;
        high = false;
    }

    return 0;
}