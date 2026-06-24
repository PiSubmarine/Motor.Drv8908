#include "PiSubmarine/Motor/Unidirectional/Drv8908/Controller.h"

namespace PiSubmarine::Motor::Unidirectional::Drv8908
{
    Controller::Controller(
        PiSubmarine::Drv8908::IDevice& chip,
        PiSubmarine::Drv8908::IPowerManager& powerManager,
        PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
        PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
        Motor::Drv8908::BridgeSide bridgeSide,
        Motor::Drv8908::Config motorConfig) :
        ControllerBase(chip, powerManager, pwmGenerator, halfBridgeMask, bridgeSide, motorConfig)
    {
    }

    Error::Api::Result<void> Controller::SetPowered(const bool enabled)
    {
        return ControllerBase::SetPowered(enabled);
    }

    Error::Api::Result<bool> Controller::IsPowered() const
    {
        return ControllerBase::IsPowered();
    }

    Error::Api::Result<NormalizedFraction> Controller::GetDutyCycle() const
    {
        return m_TargetDutyCycle;
    }

    Error::Api::Result<NormalizedFraction> Controller::GetActualDutyCycle() const
    {
        return ControllerBase::GetActualDutyCycle();
    }

    bool Controller::IsActuallyPowered() const
    {
        return ControllerBase::IsActuallyPowered();
    }

    Error::Api::Result<void> Controller::SetDutyCycle(const NormalizedFraction dutyCycle)
    {
        if (m_TargetDutyCycle == dutyCycle)
        {
            return {};
        }

        const auto oldDutyCycle = m_TargetDutyCycle;
        m_TargetDutyCycle = dutyCycle;
        NoteTargetDutyCycleChange(oldDutyCycle, m_TargetDutyCycle);
        return {};
    }

    Error::Api::Result<NormalizedFraction> Controller::GetMinimumEffectiveDutyCycle() const
    {
        return ControllerBase::GetMinimumEffectiveDutyCycle();
    }

    void Controller::Tick(const std::chrono::nanoseconds& uptime, const std::chrono::nanoseconds& deltaTime)
    {
        (void)uptime;

        auto targetDutyCycle = m_TargetDutyCycle;
        ClampTargetDutyCycleIfUnpowered(targetDutyCycle);
        if (!WantsBePowered())
        {
            m_TargetDutyCycle = 0;
        }

        if (!BeginTick())
        {
            return;
        }

        TickDriveEffort(targetDutyCycle, deltaTime);
    }

    Error::Api::Result<Telemetry::Api::State> Controller::GetState() const
    {
        const auto actualDutyCycle = ControllerBase::GetActualDutyCycle().value();
        const auto direction = actualDutyCycle == NormalizedFraction{0}
            ? Telemetry::Api::DriveDirection::Idle
            : Telemetry::Api::DriveDirection::Forward;
        return GetStateForDirection(direction);
    }
}
