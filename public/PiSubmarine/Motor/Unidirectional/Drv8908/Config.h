#pragma once

#include "PiSubmarine/NormalizedFraction.h"
#include "PiSubmarine/Motor/DutyRate.h"

namespace PiSubmarine::Motor::Drv8908
{
    struct Config
    {
        PiSubmarine::Motor::DutyRate DutyCycleChangeRate{1, std::chrono::milliseconds(100)};
        NormalizedFraction MinimalDuty{0.15};
        std::chrono::milliseconds KickDuration{100};
        std::chrono::milliseconds KickInterval{1000};
        NormalizedFraction KickDuty{0.4};

    };
}
