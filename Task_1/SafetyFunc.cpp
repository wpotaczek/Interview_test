#include "safetyFunc.h"

// this currently checks live power against power limit
void SafetyFeat::checkIOPowerDeviation(double livePower, double batPowerLimitIn, double batPowerLimitOut)
{
    if (livePower < 0)
    {
        double posLivePower = livePower * -1.0;
        this->checkPowerOutDeviation(posLivePower, batPowerLimitOut);
    }
    else
    {
        this->checkPowerInDeviation(livePower, batPowerLimitIn);
    }
}

// checks the given power against the power in limit and the allowed deviation, power should be a positive number
void SafetyFeat::checkPowerInDeviation(double livePowerIn, double batPowerLimitIn)
{
    BatteryLimits::powerIODeviationLimits_t pIOdevLimits = gBatLimits_ptr->getPowerInOutLimits();

    static uint32_t pwrInLimitTimestampPrevious = 0;
    uint32_t pwrInLimitTimestamp = gBatData_ptr->timeReceived; // Time when last frame was received
    const uint8_t maxBuffer = 3;

    static uint8_t SL1Buffer = 0;
    static uint8_t SL2Buffer = 0;
    static uint8_t SL3Buffer = 0;
    static uint8_t SL4Buffer = 0;

    // timers for SL (Safety level) initialisation
    static uint32_t SL1Timer = millis(); // mills is to get current timestamp in ms
    static const uint16_t SL1TimeLim = 1000; // ms

    static uint32_t SL2Timer = millis();
    static const uint32_t SL2TimeLim = 300000; // ms // 5 minutes

    // SL1 timed exit stuff
    static uint32_t SL1ForceOnTimer = 0;
    static const uint16_t SL1ForceOnTime = 20000; // ms // 20 seconds

    // check if we have new data
    if (pwrInLimitTimestamp != pwrInLimitTimestampPrevious)
    {
        pwrInLimitTimestampPrevious = pwrInLimitTimestamp;

        // force SL2 and or SL1 on for some time.
        //  NOW IT WILL NEVER EXIT SL3 OR 4
        if (gBiuFlags.powerInFlag == kFault)
        { // if it's not sl2 then its sl3 or 4 and we don't want to mess with the flags then
            // we know we are in SL2 here
            if (gBiuFlags.safetyLevel2.retries == 0)
            { // if we haven't retried once, keep SL2
                gBiuFlags.powerInFlag = kFault;
            }
            else
            { // if we have retried once and the power is good, go down to SL1
                gBiuFlags.powerInFlag = kWarning;
                SL1ForceOnTimer = millis(); // reset SL1 timer here just so that we have a fresh timer when entering SL1
            }
        }
        else
        {
            if (gBiuFlags.powerInFlag == kWarning)
            {
                if (livePowerIn >= batPowerLimitIn)
                    SL1ForceOnTimer = millis();
                else if ((millis() - SL1ForceOnTimer) < SL1ForceOnTime)
                {
                    gBiuFlags.powerInFlag = kWarning;
                }
                else
                {
                    gBiuFlags.powerInFlag = kNormal;
                }
            }
        }
        
        if (livePowerIn <= (batPowerLimitIn * pIOdevLimits.IN_WARNING))
        { // SL 0
            // we set SL back to SL0 (Normal) in the logic above and not here, due to the way we want to force on SL1 and SL2
            SL1Buffer = 0;
            SL2Buffer = 0;
            SL3Buffer = 0; // used as second SL2 buffer here
            SL4Buffer = 0;
        }
        else if (livePowerIn > (batPowerLimitIn * pIOdevLimits.IN_WARNING))
        { // SL 1
            if (gBiuFlags.powerInFlag <= kWarning)
            {
                if (SL1Buffer >= maxBuffer)
                {
                    gBiuioControl->SetPinValue(gBiuioControl->std_io[kErrSignal], 1); // indicate error to battery tester by setting up LED pin
                    if ((millis() - SL1Timer) > SL1TimeLim)
                    {
                        gBiuFlags.powerInFlag = kWarning; // wait for a second before going in to SL
                        SL1ForceOnTimer = millis();
                    }
                    else
                        return;
                }
                else
                {
                    SL1Timer = millis();
                    ++SL1Buffer;
                }
            }

            if (livePowerIn > (batPowerLimitIn * pIOdevLimits.IN_FAULT))
            { // SL 2, time limit for this state
                if (SL2Buffer >= maxBuffer)
                {
                    // set SL2 if we are at this stage for more than 5 minutes
                    if ((millis() - SL2Timer) > SL2TimeLim)
                        gBiuFlags.powerInFlag = kFault;
                }
                else
                {
                    SL2Timer = millis();
                    ++SL2Buffer;
                }

                if (livePowerIn > (batPowerLimitIn * pIOdevLimits.IN_CRITICAL))
                { // SL 2 as well, hard limit here
                    if (SL3Buffer >= maxBuffer)
                    {
                        // set SL 2 flag imediately if we reach this level
                        gBiuFlags.powerInFlag = kFault;
                    }
                    else
                        ++SL3Buffer;

                    if (livePowerIn > (batPowerLimitIn * pIOdevLimits.IN_DEATH))
                    { // SL 4, should never be triggered
                        if (SL4Buffer >= maxBuffer)
                        {
                            gBiuFlags.powerInFlag = kDeath;
                        }
                        else
                            ++SL4Buffer;
                    }
                    else
                        SL4Buffer = 0;
                }
                else
                {
                    SL3Buffer = 0;
                    SL4Buffer = 0;
                }
            }
            else
            {
                SL2Buffer = 0;
                SL3Buffer = 0;
                SL4Buffer = 0;
            }
        }
    }
}

// checks the given power against the power out limit and the allowed deviation, power should be a positive numbers
void SafetyFeat::checkPowerOutDeviation(double livePowerOut, double batPowerLimitOut)
{
    BatteryLimits::powerIODeviationLimits_t pIOdevLimits = gBatLimits_ptr->getPowerInOutLimits();

    static uint32_t pwrOutLimitTimestampPrevious = 0;
    uint32_t pwrOutLimitTimestamp = gBatData_ptr->timeReceived;
    const uint8_t maxBuffer = 3;

    static uint8_t SL1Buffer = 0;
    static uint8_t SL2Buffer = 0;
    static uint8_t SL3Buffer = 0; // used as second SL2 buffer here
    static uint8_t SL4Buffer = 0;

    // timers for SL initialisation
    static uint32_t SL1Timer = millis();
    static const uint16_t SL1TimeLim = 1000; // ms

    static uint32_t SL2Timer = millis();
    static const uint32_t SL2TimeLim = 300000; // ms // 5 minutes

    // SL1 timed exit stuff
    static uint32_t SL1ForceOnTimer = 0;
    static const uint16_t SL1ForceOnTime = 20000; // ms // 20 seconds

    // check if we have new data
    if (pwrOutLimitTimestamp != pwrOutLimitTimestampPrevious)
    {
        pwrOutLimitTimestampPrevious = pwrOutLimitTimestamp;
        // force SL2 and or SL1 on for some time.
        //  NOW IT WILL NEVER EXIT SL3 OR 4
        if (gBiuFlags.powerOutFlag == kFault)
        { // if it'snot sl2 then its sl3 or 4 and we don't want to mess with the flags then
            // we know we are in SL2 here
            if (gBiuFlags.safetyLevel2.retries == 0)
            { // if we haven't retried once, keep SL2
                gBiuFlags.powerOutFlag = kFault;
            }
            else
            { // if we have retried once and the power is good, go down to SL1
                gBiuFlags.powerOutFlag = kWarning;
                SL1ForceOnTimer = millis(); // reset SL1 timer here just do that we have a fresh timer when entering SL1
            }
        }
        else if (gBiuFlags.powerOutFlag == kWarning)
        {
            if (livePowerOut >= batPowerLimitOut)
                SL1ForceOnTimer = millis();
            else if ((millis() - SL1ForceOnTimer) < SL1ForceOnTime)
            {
                gBiuFlags.powerOutFlag = kWarning;
            }
            else
            {
                gBiuFlags.powerOutFlag = kNormal;
            }
        }

        if (livePowerOut <= (batPowerLimitOut * pIOdevLimits.OUT_WARNING))
        { // SL 0
            // reset buffers
            SL1Buffer = 0;
            SL2Buffer = 0;
            SL3Buffer = 0; // used as second SL2 buffer here
            SL4Buffer = 0;
        }
        else if (livePowerOut > (batPowerLimitOut * pIOdevLimits.OUT_WARNING))
        { // SL 1
            if (gBiuFlags.powerOutFlag <= kWarning)
            {
                if (SL1Buffer >= maxBuffer)
                {
                    gBiuioControl->SetPinValue(gBiuioControl->std_io[kErrSignal], 1); // indicate error to battery tester
                    if ((millis() - SL1Timer) > SL1TimeLim)
                    {
                        gBiuFlags.powerOutFlag = kWarning;
                        SL1ForceOnTimer = millis();
                    }
                }
                else
                {
                    SL1Timer = millis();
                    ++SL1Buffer;
                }
            }

            if (livePowerOut > (batPowerLimitOut * pIOdevLimits.OUT_FAULT))
            { // SL 2, time limit for this state
                if (SL2Buffer >= maxBuffer)
                {
                    // set SL2 if we are at this stage for more than 5 minutes
                    if ((millis() - SL2Timer) > SL2TimeLim)
                        gBiuFlags.powerOutFlag = kFault;
                }
                else
                {
                    SL2Timer = millis();
                    ++SL2Buffer;
                }

                if (livePowerOut > (batPowerLimitOut * pIOdevLimits.OUT_CRITICAL))
                { // SL 2 as well, hard limit here, SL2 wil initiate SL3 if retied enough times
                    if (SL3Buffer >= maxBuffer)
                    {
                        // set SL 2 flag imediately if we reach this level
                        gBiuFlags.powerOutFlag = kFault;
                    }
                    else
                        ++SL3Buffer;

                    if (livePowerOut > (batPowerLimitOut * pIOdevLimits.OUT_DEATH))
                    { // SL 4
                        if (SL4Buffer >= maxBuffer)
                        {
                            gBiuFlags.powerOutFlag = kDeath;
                        }
                        else
                            ++SL4Buffer;
                    }
                    else
                        SL4Buffer = 0;
                }
                else
                {
                    SL3Buffer = 0;
                    SL4Buffer = 0;
                }
            }
            else
            {
                SL2Buffer = 0;
                SL3Buffer = 0;
                SL4Buffer = 0;
            }
        }
    }
}
