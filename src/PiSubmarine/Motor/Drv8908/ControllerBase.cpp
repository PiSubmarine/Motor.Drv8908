#include "PiSubmarine/Motor/Drv8908/ControllerBase.h"

#include <cmath>

#include "PiSubmarine/Error/Api/MakeError.h"
#include "PiSubmarine/RegUtils.h"

namespace PiSubmarine::Motor::Drv8908
{
    namespace
    {
        [[nodiscard]] Error::Api::Result<void> MakeCommunicationError()
        {
            return std::unexpected(Error::Api::MakeError(Error::Api::ErrorCondition::CommunicationError));
        }

        [[nodiscard]] Error::Api::Result<void> MakeUnexpectedDeviceIdResult()
        {
            return std::unexpected(Error::Api::MakeError(Error::Api::ErrorCondition::DeviceError));
        }

        [[nodiscard]] Error::Api::Result<void> ValidateIcStatus(const PiSubmarine::Drv8908::IcStatus status)
        {
            if (!PiSubmarine::Drv8908::IsValid(status))
            {
                return MakeCommunicationError();
            }

            return {};
        }

        Telemetry::Api::Warnings ConvertWarnings(PiSubmarine::Drv8908::IcStatus status)
        {
            using namespace RegUtils;
            Telemetry::Api::Warnings warnings{0};
            if (HasAnyFlag(status, PiSubmarine::Drv8908::IcStatus::OverTemperatureWarning))
            {
                warnings = warnings | Telemetry::Api::Warnings::Temperature;
            }
            return warnings;
        }

        Telemetry::Api::Faults ConvertFaults(PiSubmarine::Drv8908::IcStatus status)
        {
            using namespace RegUtils;
            Telemetry::Api::Faults faults{0};
            if (HasAnyFlag(status, PiSubmarine::Drv8908::IcStatus::OpenLoad))
            {
                faults = faults | Telemetry::Api::Faults::OpenLoad;
            }
            if (HasAnyFlag(status, PiSubmarine::Drv8908::IcStatus::OverVoltage))
            {
                faults = faults | Telemetry::Api::Faults::Overvoltage;
            }
            if (HasAnyFlag(status, PiSubmarine::Drv8908::IcStatus::UnderVoltage))
            {
                faults = faults | Telemetry::Api::Faults::Undervoltage;
            }
            if (HasAnyFlag(status, PiSubmarine::Drv8908::IcStatus::OverCurrent))
            {
                faults = faults | Telemetry::Api::Faults::Overcurrent;
            }
            if (HasAnyFlag(status, PiSubmarine::Drv8908::IcStatus::OverTemperatureShutdown))
            {
                faults = faults | Telemetry::Api::Faults::Overtemperature;
            }
            return faults;
        }
    }

    ControllerBase::ControllerBase(
        PiSubmarine::Drv8908::IDevice& chip,
        PiSubmarine::Drv8908::IPowerManager& powerManager,
        PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
        PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
        BridgeSide initialBridgeSide,
        Config motorConfig) :
        ControllerBase(
            chip,
            powerManager,
            pwmGenerator,
            initialBridgeSide == BridgeSide::High ? halfBridgeMask : PiSubmarine::Drv8908::HalfBridgeBitMask{0},
            initialBridgeSide == BridgeSide::Low ? halfBridgeMask : PiSubmarine::Drv8908::HalfBridgeBitMask{0},
            motorConfig)
    {
    }

    ControllerBase::ControllerBase(
        PiSubmarine::Drv8908::IDevice& chip,
        PiSubmarine::Drv8908::IPowerManager& powerManager,
        PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
        PiSubmarine::Drv8908::HalfBridgeBitMask initialHighSideHalfBridgeMask,
        PiSubmarine::Drv8908::HalfBridgeBitMask initialLowSideHalfBridgeMask,
        Config motorConfig) :
        m_Chip(chip),
        m_PowerManager(powerManager),
        m_PwmGenerator(pwmGenerator),
        m_HalfBridges(static_cast<PiSubmarine::Drv8908::HalfBridgeBitMask>(
            RegUtils::ToInt(initialHighSideHalfBridgeMask) | RegUtils::ToInt(initialLowSideHalfBridgeMask))),
        m_RequestedHighSideHalfBridges(initialHighSideHalfBridgeMask),
        m_RequestedLowSideHalfBridges(initialLowSideHalfBridgeMask),
        m_MotorConfig(motorConfig)
    {
    }

    Error::Api::Result<void> ControllerBase::SetPowered(bool enabled)
    {
        m_WantsBePowered = enabled;
        return {};
    }

    Error::Api::Result<bool> ControllerBase::IsPowered() const
    {
        return m_WantsBePowered;
    }

    Error::Api::Result<NormalizedFraction> ControllerBase::GetActualDutyCycle() const
    {
        return m_CurrentDutyCycle;
    }

    bool ControllerBase::IsActuallyPowered() const
    {
        return m_PowerLease.IsValid();
    }

    Error::Api::Result<NormalizedFraction> ControllerBase::GetMinimumEffectiveDutyCycle() const
    {
        return m_MotorConfig.MinimalDuty;
    }

    bool ControllerBase::WantsBePowered() const
    {
        return m_WantsBePowered;
    }

    void ControllerBase::ClampTargetDutyCycleIfUnpowered(NormalizedFraction& targetDutyCycle) const
    {
        if (!m_WantsBePowered)
        {
            targetDutyCycle = 0;
        }
    }

    void ControllerBase::NoteTargetDutyCycleChange(
        const NormalizedFraction oldDutyCycle,
        const NormalizedFraction newDutyCycle)
    {
        if (newDutyCycle < m_MotorConfig.KickDuty && oldDutyCycle <= m_MotorConfig.MinimalDuty)
        {
            m_KickNeeded = true;
        }
    }

    bool ControllerBase::BeginTick()
    {
        if (m_WantsBePowered != m_PowerLease.IsValid())
        {
            if (m_WantsBePowered)
            {
                const auto powerUpResult = PowerUp();
                if (!powerUpResult.has_value())
                {
                    m_OperationalState = powerUpResult.error().Condition == Error::Api::ErrorCondition::CommunicationError
                        ? Telemetry::Api::OperationalState::Faulted
                        : Telemetry::Api::OperationalState::Degraded;
                    m_Faults = Telemetry::Api::Faults{0};
                    m_Warnings = Telemetry::Api::Warnings{0};
                    if (m_PowerLease.IsValid())
                    {
                        m_PowerManager.Release(m_PowerLease);
                    }
                    return false;
                }
            }
            else if (m_CurrentDutyCycle == NormalizedFraction{0})
            {
                m_PowerManager.Release(m_PowerLease);
            }
        }

        if (m_PowerLease.IsValid())
        {
            const auto readStatusResult = ReadStatus();
            if (!readStatusResult.has_value())
            {
                m_OperationalState = readStatusResult.error().Condition == Error::Api::ErrorCondition::CommunicationError
                    ? Telemetry::Api::OperationalState::Faulted
                    : Telemetry::Api::OperationalState::Degraded;
                m_Faults = Telemetry::Api::Faults{0};
                m_Warnings = Telemetry::Api::Warnings{0};
                return false;
            }
            return true;
        }

        return false;
    }

    void ControllerBase::TickDriveEffort(
        const NormalizedFraction targetDutyCycle,
        const std::chrono::nanoseconds& deltaTime)
    {
        if (targetDutyCycle == NormalizedFraction{0} && m_State != ControlState::Normal)
        {
            m_State = ControlState::Normal;
            m_TimeSinceKickTransition = std::chrono::nanoseconds{0};
        }

        if (m_State == ControlState::Normal)
        {
            auto transitionTarget = targetDutyCycle;
            if (targetDutyCycle < m_MotorConfig.MinimalDuty)
            {
                transitionTarget = 0;
            }
            const auto transitionRate = transitionTarget >= m_CurrentDutyCycle
                ? m_MotorConfig.DutyCycleIncreaseChangeRate
                : m_MotorConfig.DutyCycleDecreaseChangeRate;
            if (!TransitionDutyCycle(transitionTarget, transitionRate, deltaTime).has_value())
            {
                return;
            }

            if (targetDutyCycle >= m_MotorConfig.MinimalDuty)
            {
                if (
                    (m_TimeSinceKickTransition >= m_MotorConfig.KickInterval && m_MotorConfig.KickInterval.count() > 0)
                    || m_KickNeeded)
                {
                    m_State = ControlState::KickRise;
                    m_TimeSinceKickTransition = std::chrono::nanoseconds(0);
                    m_KickNeeded = false;
                }
            }
        }
        else if (m_State == ControlState::KickRise)
        {
            if (!TransitionDutyCycle(m_MotorConfig.KickDuty, m_MotorConfig.KickDutyCycleChangeRate, deltaTime).has_value())
            {
                return;
            }

            if (m_TimeSinceKickTransition >= m_MotorConfig.KickDuration / 2)
            {
                m_State = ControlState::KickFall;
                m_TimeSinceKickTransition = std::chrono::nanoseconds(0);
            }
        }
        else if (m_State == ControlState::KickFall)
        {
            if (targetDutyCycle >= m_MotorConfig.KickDuty)
            {
                m_State = ControlState::Normal;
                m_TimeSinceKickTransition = std::chrono::nanoseconds(0);
            }
            else
            {
                if (!TransitionDutyCycle(targetDutyCycle, m_MotorConfig.KickDutyCycleChangeRate, deltaTime).has_value())
                {
                    return;
                }

                if (m_TimeSinceKickTransition >= m_MotorConfig.KickDuration / 2)
                {
                    m_State = ControlState::Normal;
                    m_TimeSinceKickTransition = std::chrono::nanoseconds(0);
                }
            }
        }

        m_TimeSinceKickTransition += deltaTime;
    }

    Error::Api::Result<void> ControllerBase::SetBridgeSide(const BridgeSide bridgeSide)
    {
        return SetHalfBridgeStates(
            bridgeSide == BridgeSide::High ? m_HalfBridges : PiSubmarine::Drv8908::HalfBridgeBitMask{0},
            bridgeSide == BridgeSide::Low ? m_HalfBridges : PiSubmarine::Drv8908::HalfBridgeBitMask{0});
    }

    Error::Api::Result<void> ControllerBase::SetHalfBridgeStates(
        const PiSubmarine::Drv8908::HalfBridgeBitMask highSideHalfBridgeMask,
        const PiSubmarine::Drv8908::HalfBridgeBitMask lowSideHalfBridgeMask)
    {
        m_RequestedHighSideHalfBridges = highSideHalfBridgeMask;
        m_RequestedLowSideHalfBridges = lowSideHalfBridgeMask;

        if (!m_PowerLease.IsValid())
        {
            return {};
        }

        return ApplyHalfBridgeStates(m_CurrentDutyCycle);
    }

    Error::Api::Result<Telemetry::Api::State> ControllerBase::GetStateForDirection(
        const Telemetry::Api::DriveDirection direction) const
    {
        return Telemetry::Api::State{
            .Operational = m_OperationalState,
            .ActiveFaults = m_Faults,
            .ActiveWarnings = m_Warnings,
            .Direction = direction,
            .DriveEffort = m_CurrentDutyCycle};
    }

    void ControllerBase::RequestKick()
    {
        m_KickNeeded = true;
    }

    Error::Api::Result<void> ControllerBase::PowerUp()
    {
        using namespace RegUtils;

        m_PowerLease = m_PowerManager.Acquire();

        PiSubmarine::Drv8908::IcStatus icStat;
        auto status = m_Chip.GetStatus(icStat);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }

        PiSubmarine::Drv8908::ConfigCtrl configCtrl{};
        status = m_Chip.GetConfigCtrl(configCtrl);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }

        if (configCtrl.Id != PiSubmarine::Drv8908::IcId::DRV8908)
        {
            return MakeUnexpectedDeviceIdResult();
        }

        status = m_Chip.SetEnabledOpenLoadDetect(m_HalfBridges);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        status = m_Chip.SetOpenLoadDetectControl3(PiSubmarine::Drv8908::OcpDeglitchTime::MicroSeconds60, true);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        status = m_Chip.SetOpenLoadDetectControl2(
            PiSubmarine::Drv8908::OpenLoadDetectControl::OldRep | PiSubmarine::Drv8908::OpenLoadDetectControl::OldOp);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        PiSubmarine::Drv8908::PwmGeneratorBitMask pwmGenerators;
        status = m_Chip.GetEnabledPwmGenerators(pwmGenerators);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        pwmGenerators = pwmGenerators | PiSubmarine::Drv8908::ToPwmGeneratorBitMask(m_PwmGenerator);
        status = m_Chip.SetEnabledPwmGenerators(pwmGenerators);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        status = m_Chip.SetPwmFrequency(m_PwmGenerator, PiSubmarine::Drv8908::PwmFrequency::Hz2000);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        PiSubmarine::Drv8908::HalfBridgeBitMask pwmHalfBridges;
        status = m_Chip.GetHalfBridgePwmModes(pwmHalfBridges);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        pwmHalfBridges = pwmHalfBridges | m_HalfBridges;
        status = m_Chip.SetHalfBridgePwmModes(pwmHalfBridges);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }

        const auto applyHalfBridgeStatesResult = ApplyHalfBridgeStates(m_CurrentDutyCycle);
        if (!applyHalfBridgeStatesResult.has_value())
        {
            return applyHalfBridgeStatesResult;
        }

        status = m_Chip.SetPwmMap(m_HalfBridges, m_PwmGenerator);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }

        // TODO AFAIK makes sense only for bi-directional motors
        /*
        status = m_Chip.SetHalfBridgeActiveFreeWheeling(m_HalfBridges);
        if (!ValidateIcStatus(status).has_value())
        {
            return MakeCommunicationError();
        }
        */

        m_KickNeeded = true;
        m_OperationalState = Telemetry::Api::OperationalState::Operational;
        return {};
    }

    Error::Api::Result<void> ControllerBase::ReadStatus()
    {
        PiSubmarine::Drv8908::IcStatus stat;

        PiSubmarine::Drv8908::IcStatus chipStatus;
        stat = m_Chip.GetStatus(chipStatus);
        if (!PiSubmarine::Drv8908::IsValid(stat))
        {
            return MakeCommunicationError();
        }
        m_Faults = ConvertFaults(chipStatus);
        m_Warnings = ConvertWarnings(chipStatus);
        if (m_Faults != Telemetry::Api::Faults{0} || m_Warnings != Telemetry::Api::Warnings{0})
        {
            m_OperationalState = Telemetry::Api::OperationalState::Degraded;
            PiSubmarine::Drv8908::ConfigCtrl configCtrl{};
            stat = m_Chip.GetConfigCtrl(configCtrl);
            if (!PiSubmarine::Drv8908::IsValid(stat))
            {
                return MakeCommunicationError();
            }
            configCtrl.ClrFlt = true;
            stat = m_Chip.SetConfigCtrl(configCtrl);
            if (!PiSubmarine::Drv8908::IsValid(stat))
            {
                return MakeCommunicationError();
            }
        }
        else
        {
            m_OperationalState = Telemetry::Api::OperationalState::Operational;
        }

        return {};
    }

    Error::Api::Result<void> ControllerBase::TransitionDutyCycle(
        const NormalizedFraction targetDutyCycle,
        DutyRate speed,
        const std::chrono::nanoseconds deltaTime)
    {
        if (m_CurrentDutyCycle != targetDutyCycle)
        {
            if (speed.DutyPerSecond <= 0.0)
            {
                return SetDutyCycleInternal(targetDutyCycle);
            }

            double dutyDeltaCurrent = std::fabs(targetDutyCycle - m_CurrentDutyCycle);
            double dutyDeltaTick = speed * deltaTime;
            if (std::fabs(dutyDeltaTick) > dutyDeltaCurrent)
            {
                dutyDeltaTick = dutyDeltaCurrent;
            }

            auto newDutyCycle = m_CurrentDutyCycle;
            if (m_CurrentDutyCycle < targetDutyCycle)
            {
                newDutyCycle = m_CurrentDutyCycle + dutyDeltaTick;
            }
            else if (m_CurrentDutyCycle > targetDutyCycle)
            {
                newDutyCycle = m_CurrentDutyCycle - dutyDeltaTick;
            }

            return SetDutyCycleInternal(newDutyCycle);
        }

        return {};
    }

    Error::Api::Result<void> ControllerBase::SetDutyCycleInternal(const NormalizedFraction dutyCycle)
    {
        if (!m_PowerLease.IsValid())
        {
            return std::unexpected(Error::Api::MakeError(Error::Api::ErrorCondition::ContractError));
        }

        const auto halfBridgeStateResult = ApplyHalfBridgeStates(dutyCycle);
        if (!halfBridgeStateResult.has_value())
        {
            return halfBridgeStateResult;
        }

        const NormalizedIntFraction<8> dutyCycleInt{static_cast<double>(dutyCycle)};
        const auto stat = m_Chip.SetDutyCycle(m_PwmGenerator, dutyCycleInt);
        if (!PiSubmarine::Drv8908::IsValid(stat))
        {
            m_OperationalState = Telemetry::Api::OperationalState::Faulted;
            return MakeCommunicationError();
        }

        m_CurrentDutyCycle = dutyCycle;
        return {};
    }

    bool ControllerBase::ShouldDrive(const NormalizedFraction dutyCycle) const
    {
        return dutyCycle >= m_MotorConfig.MinimalDuty;
    }

    Error::Api::Result<void> ControllerBase::ApplyHalfBridgeStates(const NormalizedFraction dutyCycle) const
    {
        using namespace RegUtils;

        const auto highSideHalfBridges = ShouldDrive(dutyCycle)
            ? m_RequestedHighSideHalfBridges
            : PiSubmarine::Drv8908::HalfBridgeBitMask{0};
        const auto lowSideHalfBridges = ShouldDrive(dutyCycle)
            ? m_RequestedLowSideHalfBridges
            : PiSubmarine::Drv8908::HalfBridgeBitMask{0};

        const auto activeHalfBridges = highSideHalfBridges | lowSideHalfBridges;
        const auto inactiveHalfBridges = static_cast<PiSubmarine::Drv8908::HalfBridgeBitMask>(
            ToInt(m_HalfBridges) & ~ToInt(activeHalfBridges));

        if (highSideHalfBridges != PiSubmarine::Drv8908::HalfBridgeBitMask{0})
        {
            const auto status = m_Chip.SetHalfBridgeEnabled(highSideHalfBridges, true, false);
            if (!PiSubmarine::Drv8908::IsValid(status))
            {
                return MakeCommunicationError();
            }
        }

        if (lowSideHalfBridges != PiSubmarine::Drv8908::HalfBridgeBitMask{0})
        {
            const auto status = m_Chip.SetHalfBridgeEnabled(lowSideHalfBridges, false, true);
            if (!PiSubmarine::Drv8908::IsValid(status))
            {
                return MakeCommunicationError();
            }
        }

        if (inactiveHalfBridges != PiSubmarine::Drv8908::HalfBridgeBitMask{0})
        {
            const auto status = m_Chip.SetHalfBridgeEnabled(inactiveHalfBridges, false, false);
            if (!PiSubmarine::Drv8908::IsValid(status))
            {
                return MakeCommunicationError();
            }
        }

        return {};
    }
}
