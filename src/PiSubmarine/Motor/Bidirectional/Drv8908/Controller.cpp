#include "PiSubmarine/Motor/Bidirectional/Drv8908/Controller.h"

#include <cmath>

namespace PiSubmarine::Motor::Bidirectional::Drv8908
{
    Controller::Controller(
        PiSubmarine::Drv8908::IDevice& chip,
        PiSubmarine::Drv8908::IPowerManager& powerManager,
        PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
        PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
        const Motor::Drv8908::BridgeSide forwardBridgeSide) :
        Controller(chip, powerManager, pwmGenerator, halfBridgeMask, forwardBridgeSide, Motor::Drv8908::Config{})
    {
    }

    Controller::Controller(
        PiSubmarine::Drv8908::IDevice& chip,
        PiSubmarine::Drv8908::IPowerManager& powerManager,
        PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
        PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
        const Motor::Drv8908::BridgeSide forwardBridgeSide,
        const Motor::Drv8908::Config motorConfig) :
        ControllerBase(chip, powerManager, pwmGenerator, halfBridgeMask, forwardBridgeSide, motorConfig),
        m_ForwardBridgeSide(forwardBridgeSide)
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

    Error::Api::Result<SignedNormalizedFraction> Controller::GetDutyCycle() const
    {
        return m_TargetDutyCycle;
    }

    Error::Api::Result<void> Controller::SetDutyCycle(const SignedNormalizedFraction dutyCycle)
    {
        if (static_cast<double>(m_TargetDutyCycle) == static_cast<double>(dutyCycle))
        {
            return {};
        }

        const auto oldMagnitude = GetMagnitude(m_TargetDutyCycle);
        const auto oldDirection = GetDirection(m_TargetDutyCycle);
        m_TargetDutyCycle = dutyCycle;
        NoteTargetDutyCycleChange(oldMagnitude, GetMagnitude(m_TargetDutyCycle));
        if (oldDirection != GetDirection(m_TargetDutyCycle) && GetDirection(m_TargetDutyCycle) != Telemetry::Api::DriveDirection::Idle)
        {
            RequestKick();
        }
        return {};
    }

    Error::Api::Result<NormalizedFraction> Controller::GetForwardMinimalEffectiveDutyCycle() const
    {
        return ControllerBase::GetMinimumEffectiveDutyCycle();
    }

    Error::Api::Result<NormalizedFraction> Controller::GetReverseMinimalEffectiveDutyCycle() const
    {
        return ControllerBase::GetMinimumEffectiveDutyCycle();
    }

    void Controller::Tick(const std::chrono::nanoseconds& uptime, const std::chrono::nanoseconds& deltaTime)
    {
        (void)uptime;

        if (!WantsBePowered())
        {
            m_TargetDutyCycle = SignedNormalizedFraction{0};
        }

        if (!BeginTick())
        {
            return;
        }

        const auto targetDirection = GetDirection(m_TargetDutyCycle);
        const auto targetMagnitude = GetMagnitude(m_TargetDutyCycle);
        const auto currentMagnitude = ControllerBase::GetActualDutyCycle().value();

        if (m_CurrentDirection != targetDirection && currentMagnitude > NormalizedFraction{0})
        {
            TickDriveEffort(NormalizedFraction{0}, deltaTime);

            if (ControllerBase::GetActualDutyCycle().value() == NormalizedFraction{0})
            {
                if (targetDirection == Telemetry::Api::DriveDirection::Idle)
                {
                    m_CurrentDirection = Telemetry::Api::DriveDirection::Idle;
                }
                else
                {
                    ApplyDirection(targetDirection);
                }
            }
            return;
        }

        if (m_CurrentDirection != targetDirection && targetDirection != Telemetry::Api::DriveDirection::Idle)
        {
            ApplyDirection(targetDirection);
        }

        TickDriveEffort(targetMagnitude, deltaTime);

        if (ControllerBase::GetActualDutyCycle().value() == NormalizedFraction{0}
            && targetDirection == Telemetry::Api::DriveDirection::Idle)
        {
            m_CurrentDirection = Telemetry::Api::DriveDirection::Idle;
        }
    }

    Error::Api::Result<Telemetry::Api::State> Controller::GetState() const
    {
        const auto direction = ControllerBase::GetActualDutyCycle().value() == NormalizedFraction{0}
            ? Telemetry::Api::DriveDirection::Idle
            : m_CurrentDirection;
        return GetStateForDirection(direction);
    }

    NormalizedFraction Controller::GetMagnitude(const SignedNormalizedFraction dutyCycle)
    {
        return NormalizedFraction{std::abs(static_cast<double>(dutyCycle))};
    }

    Telemetry::Api::DriveDirection Controller::GetDirection(const SignedNormalizedFraction dutyCycle)
    {
        const auto value = static_cast<double>(dutyCycle);
        if (value > 0.0)
        {
            return Telemetry::Api::DriveDirection::Forward;
        }
        if (value < 0.0)
        {
            return Telemetry::Api::DriveDirection::Reverse;
        }
        return Telemetry::Api::DriveDirection::Idle;
    }

    Motor::Drv8908::BridgeSide Controller::ReverseBridgeSide(const Motor::Drv8908::BridgeSide bridgeSide)
    {
        return bridgeSide == Motor::Drv8908::BridgeSide::High
            ? Motor::Drv8908::BridgeSide::Low
            : Motor::Drv8908::BridgeSide::High;
    }

    void Controller::ApplyDirection(const Telemetry::Api::DriveDirection direction)
    {
        if (direction == Telemetry::Api::DriveDirection::Forward)
        {
            SetBridgeSide(m_ForwardBridgeSide);
        }
        else if (direction == Telemetry::Api::DriveDirection::Reverse)
        {
            SetBridgeSide(ReverseBridgeSide(m_ForwardBridgeSide));
        }

        m_CurrentDirection = direction;
        RequestKick();
    }
}
