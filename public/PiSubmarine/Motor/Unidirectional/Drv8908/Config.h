#pragma once

#include "PiSubmarine/NormalizedFraction.h"
#include "PiSubmarine/Motor/DutyRate.h"

namespace PiSubmarine::Motor::Drv8908
{
    struct Config
    {
        DutyRate DutyCycleChangeRate{1, std::chrono::milliseconds(100)};
        NormalizedFraction MinimalDuty{0.20};
        std::chrono::milliseconds KickDuration{100};
        std::chrono::milliseconds KickInterval{0};
        NormalizedFraction KickDuty{0.4};
        DutyRate KickDutyCycleChangeRate{1, std::chrono::milliseconds(10)};
    };
}
