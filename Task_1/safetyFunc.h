#pragma once
#include <map>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

enum DegradationLevels
{
    kNormal,
    kWarning,
    kFault,
    kCritical,
    kDeath,
    kLevelsNum,
};

enum StdPins
{
    kEStop,
    kOnboardLed,
    kErrSignal,
    kWdog,
    kExtPwrSens,
    kStdPinsNum,
};

class SafetyFeat
{
public:
    virtual void checkIOPowerDeviation(double livePower, double batPowerLimitIn, double batPowerLimitOut);
    virtual void checkPowerInDeviation(double livePowerIn, double batPowerLimitIn);
    virtual void checkPowerOutDeviation(double livePowerOut, double batPowerLimitOut);
};


class BatteryLimits
{
public:
    struct paramDetails
    {
        int16_t limit = 0;
        uint8_t retries = 0;
    };
    /**
     * @brief Struct holding Temperature limits
     * @details Unit: degree celcius
     */

    struct packTempLimits_t
    {
        std::map<enum DegradationLevels, struct paramDetails> highTemp = {
            {kWarning,      {0,0},},
            {kFault,        {0,0},},
            {kCritical,     {0,0},},
            {kDeath,        {0,0},},
        };

        std::map<enum DegradationLevels, struct paramDetails> lowTemp = {
            {kWarning,      {0,0},},
            {kFault,        {0,0},},
            {kCritical,     {0,0},},
            {kDeath,        {0,0},},
        };

        const uint8_t maxRetries = 3;
        const uint8_t riMaxRetries = 3;
        const uint16_t riCheckInterval = 6000;

        uint16_t riTemp = 0;     ///< rapid increase temp
        uint32_t riInterval = 0; ///< rapid increase time window
    };

    /// @brief cell voltage limits
    /// @details Unit: mV
    struct cellVoltageLimits_t
    {
        std::map<enum DegradationLevels, struct paramDetails> highVoltage = {
            {kWarning,      {0,0},},
            {kFault,        {0,0},},
            {kCritical,     {0,0},},
            {kDeath,        {0,0},},
        };

        std::map<enum DegradationLevels, struct paramDetails> lowVoltage = {
            {kWarning,      {0,0},},
            {kFault,        {0,0},},
            {kCritical,     {0,0},},
            {kDeath,        {0,0},},
        };
        const uint16_t maxRetries = 3;
        const double SCALE_FACTOR = 0.001; ///< multiply limit value with the scale factor to get the value in Volts
    };

    /// @brief Cell voltage deviation limits NOT in use
    /// @details Sets a limit for how much deviation there can be between the voltage
    ///     of the max and min cell.
    ///     Unit: mV
    /// @deprecated since forever
    struct cellVoltageDeviationLimits_t
    {
        uint16_t WARNING;
        uint16_t FAULT;
        uint16_t CRITICAL;
        uint16_t DEATH;
        const double SCALE_FACTOR = 0.001;
    };

    /// @brief The power limit for power in/out of the battery
    /// @details Set as a percentage, represented as a decimal.
    struct powerIODeviationLimits_t
    {
        std::map<enum DegradationLevels, struct paramDetails> highVoltage = {
            {kWarning,      {0,0},},
            {kFault,        {0,0},},
            {kCritical,     {0,0},},
            {kDeath,        {0,0},},
        };

        std::map<enum DegradationLevels, struct paramDetails> lowVoltage = {
            {kWarning,      {0,0},},
            {kFault,        {0,0},},
            {kCritical,     {0,0},},
            {kDeath,        {0,0},},
        };
        
        // limits for power in
        double IN_WARNING;
        double IN_FAULT;
        double IN_CRITICAL;
        double IN_DEATH;
        // Limits for power out
        double OUT_WARNING = 1234;
        double OUT_FAULT = 2345;
        double OUT_CRITICAL = 2346;
        double OUT_DEATH;
    };

    /// @brief Time to wait in the opening and closing relays procedure. NOT in use
    /// @details Unit: mS
    struct relayOperationWaitTimes_t
    {
        uint16_t FROM_PCR_ON_TO_FIRST_MSG; 
        uint16_t FROM_PCR_ON_TO_NR_ON;    
    };

    /// @brief For converting bat values to the format in the BIU.
    enum BIU_INTERNAL_VALUES
    {
        PACK_VOLTAGE,
        CELL_VOLTAGE,
        INS_RES
    };

    /**
     * @name Other Limits and constant values
     * @brief a collection of limits and values used with limits that didn't fit in their own struct
     */
    int OpenRelayCurrentLimit; ///< The max limit for the current in/out of the battery when opening/closing contactors
    int SL1MaxCurrentLimit; ///< The max limit for the current in/out of the battery when in safety level 1. (just if the limit is decided so)
    double SL1CurrentPercReduction; ///< Percent reduction in current for SL1, if we decide to set that limit (not all SL1's have this)
    int SL2MaxTimeBetweenRetries;
    uint32_t SL2MaxRetryAdjustTime;
    uint8_t SL2MaxNrRetries;
    uint32_t batCommTimeoutLimit;
    uint8_t batCRCFailsLimit;
    uint32_t batPRUNTimeoutLimit;
    uint8_t SL2CurrentTollerance;
    /// @brief Insulation resistance limit
    /// @details has only one level. values above this level are ok, anything under is not.
    ///     Unit: mV
    double insulationResistanceCriticalLimit;

protected:
    powerIODeviationLimits_t powerIoDevLimits;

public:
    powerIODeviationLimits_t getPowerInOutLimits() {return this->powerIoDevLimits;};
};

class BIUFlags
{
public:
    DegradationLevels powerOutFlag = kNormal;
    DegradationLevels powerInFlag = kNormal;
};
