#pragma once

#include "PiSubmarine/Motor/DutyRate.h"
#include "PiSubmarine/NormalizedFraction.h"

namespace PiSubmarine::Motor::Drv8908
{
    struct Config
    {
        DutyRate DutyCycleIncreaseChangeRate{1, std::chrono::milliseconds(100)};
        DutyRate DutyCycleDecreaseChangeRate{0, std::chrono::milliseconds(100)};
        NormalizedFraction MinimalDuty{0.20};
        std::chrono::milliseconds KickDuration{250};
        std::chrono::milliseconds KickInterval{0};
        NormalizedFraction KickDuty{0.50};
        DutyRate KickDutyCycleChangeRate{1, std::chrono::milliseconds(10)};
    };
}
