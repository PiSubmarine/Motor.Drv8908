#pragma once

#include <stdexcept>

#include "PiSubmarine/Drv8908/HalfBridgeBitMask.h"
#include "PiSubmarine/Drv8908/IDevice.h"
#include "PiSubmarine/Drv8908/IPowerManager.h"
#include "PiSubmarine/Drv8908/PwmGenerator.h"
#include "PiSubmarine/Motor/Bidirectional/Api/IController.h"
#include "PiSubmarine/Motor/Drv8908/BridgeSide.h"
#include "PiSubmarine/Motor/Drv8908/Config.h"
#include "PiSubmarine/Motor/Drv8908/ControllerBase.h"

namespace PiSubmarine::Motor::Bidirectional::Drv8908
{
    class Controller : public Motor::Bidirectional::Api::IController, public Motor::Drv8908::ControllerBase
    {
    public:
        Controller(
            PiSubmarine::Drv8908::IDevice& chip,
            PiSubmarine::Drv8908::IPowerManager& powerManager,
            PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
            PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
            Motor::Drv8908::BridgeSide forwardBridgeSide);

        Controller(
            PiSubmarine::Drv8908::IDevice& chip,
            PiSubmarine::Drv8908::IPowerManager& powerManager,
            PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
            PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
            Motor::Drv8908::BridgeSide forwardBridgeSide,
            Motor::Drv8908::Config motorConfig);

        Error::Api::Result<void> SetPowered(bool enabled) override;
        [[nodiscard]] Error::Api::Result<bool> IsPowered() const override;
        [[nodiscard]] Error::Api::Result<SignedNormalizedFraction> GetDutyCycle() const override;
        Error::Api::Result<void> SetDutyCycle(SignedNormalizedFraction dutyCycle) override;
        Error::Api::Result<NormalizedFraction> GetForwardMinimalEffectiveDutyCycle() const override;
        Error::Api::Result<NormalizedFraction> GetReverseMinimalEffectiveDutyCycle() const override;

        void Tick(const std::chrono::nanoseconds& uptime, const std::chrono::nanoseconds& deltaTime) override;

        [[nodiscard]] Error::Api::Result<Telemetry::Api::State> GetState() const override;

    private:
        Motor::Drv8908::BridgeSide m_ForwardBridgeSide;
        SignedNormalizedFraction m_TargetDutyCycle{0};
        Telemetry::Api::DriveDirection m_CurrentDirection{Telemetry::Api::DriveDirection::Idle};

        [[nodiscard]] static NormalizedFraction GetMagnitude(SignedNormalizedFraction dutyCycle);
        [[nodiscard]] static Telemetry::Api::DriveDirection GetDirection(SignedNormalizedFraction dutyCycle);
        [[nodiscard]] static Motor::Drv8908::BridgeSide ReverseBridgeSide(Motor::Drv8908::BridgeSide bridgeSide);
        void ApplyDirection(Telemetry::Api::DriveDirection direction);
    };
}
