#pragma once

#include <stdexcept>

#include "PiSubmarine/Drv8908/HalfBridgeBitMask.h"
#include "PiSubmarine/Drv8908/IDevice.h"
#include "PiSubmarine/Drv8908/IPowerManager.h"
#include "PiSubmarine/Drv8908/PwmGenerator.h"
#include "PiSubmarine/Motor/Drv8908/BridgeSide.h"
#include "PiSubmarine/Motor/Drv8908/Config.h"
#include "PiSubmarine/Motor/Drv8908/ControllerBase.h"
#include "PiSubmarine/Motor/Unidirectional/Api/IController.h"


namespace PiSubmarine::Motor::Unidirectional::Drv8908
{
    class Controller : public Motor::Unidirectional::Api::IController, public Motor::Drv8908::ControllerBase
    {
    public:
        Controller(
            PiSubmarine::Drv8908::IDevice& chip,
            PiSubmarine::Drv8908::IPowerManager& powerManager,
            PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
            PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
            Motor::Drv8908::BridgeSide bridgeSide,
            Motor::Drv8908::Config motorConfig
        );

        Error::Api::Result<void> SetPowered(bool enabled) override;
        [[nodiscard]] Error::Api::Result<bool> IsPowered() const override;
        [[nodiscard]] Error::Api::Result<NormalizedFraction> GetDutyCycle() const override;
        [[nodiscard]] Error::Api::Result<NormalizedFraction> GetActualDutyCycle() const override;
        [[nodiscard]] bool IsActuallyPowered() const override;
        Error::Api::Result<void> SetDutyCycle(NormalizedFraction dutyCycle) override;
        Error::Api::Result<NormalizedFraction> GetMinimumEffectiveDutyCycle() const override;
        void Tick(const std::chrono::nanoseconds& uptime, const std::chrono::nanoseconds& deltaTime) override;
        [[nodiscard]] Error::Api::Result<Telemetry::Api::State> GetState() const override;

    private:
        NormalizedFraction m_TargetDutyCycle{0};
    };
}
