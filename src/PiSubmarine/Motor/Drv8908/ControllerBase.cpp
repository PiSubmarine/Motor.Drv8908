#include "PiSubmarine/Motor/Drv8908/ControllerBase.h"

#include <cassert>
#include <cmath>

#include "PiSubmarine/RegUtils.h"

namespace PiSubmarine::Motor::Drv8908
{
    namespace
    {
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
        m_HighSideHalfBridges(initialHighSideHalfBridgeMask),
        m_LowSideHalfBridges(initialLowSideHalfBridgeMask),
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
                PowerUp();
            }
            else if (m_CurrentDutyCycle == NormalizedFraction{0})
            {
                m_PowerManager.Release(m_PowerLease);
            }
        }

        if (m_PowerLease.IsValid())
        {
            ReadStatus();
            if (m_OperationalState == Telemetry::Api::OperationalState::Faulted)
            {
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
            TransitionDutyCycle(transitionTarget, m_MotorConfig.DutyCycleChangeRate, deltaTime);

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
            TransitionDutyCycle(m_MotorConfig.KickDuty, m_MotorConfig.KickDutyCycleChangeRate, deltaTime);

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
                TransitionDutyCycle(targetDutyCycle, m_MotorConfig.KickDutyCycleChangeRate, deltaTime);

                if (m_TimeSinceKickTransition >= m_MotorConfig.KickDuration / 2)
                {
                    m_State = ControlState::Normal;
                    m_TimeSinceKickTransition = std::chrono::nanoseconds(0);
                }
            }
        }

        m_TimeSinceKickTransition += deltaTime;
    }

    void ControllerBase::SetBridgeSide(const BridgeSide bridgeSide)
    {
        SetHalfBridgeStates(
            bridgeSide == BridgeSide::High ? m_HalfBridges : PiSubmarine::Drv8908::HalfBridgeBitMask{0},
            bridgeSide == BridgeSide::Low ? m_HalfBridges : PiSubmarine::Drv8908::HalfBridgeBitMask{0});
    }

    void ControllerBase::SetHalfBridgeStates(
        const PiSubmarine::Drv8908::HalfBridgeBitMask highSideHalfBridgeMask,
        const PiSubmarine::Drv8908::HalfBridgeBitMask lowSideHalfBridgeMask)
    {
        m_HighSideHalfBridges = highSideHalfBridgeMask;
        m_LowSideHalfBridges = lowSideHalfBridgeMask;

        if (!m_PowerLease.IsValid())
        {
            return;
        }

        ApplyHalfBridgeStates();
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

    void ControllerBase::PowerUp()
    {
        using namespace RegUtils;

        m_PowerLease = m_PowerManager.Acquire();

        PiSubmarine::Drv8908::IcStatus icStat;
        auto status = m_Chip.GetStatus(icStat);
        assert(IsValid(status));

        PiSubmarine::Drv8908::ConfigCtrl configCtrl{};
        if (!PiSubmarine::Drv8908::IsValid(m_Chip.GetConfigCtrl(configCtrl)))
        {
            m_OperationalState = Telemetry::Api::OperationalState::Faulted;
            return;
        }

        if (configCtrl.Id != PiSubmarine::Drv8908::IcId::DRV8908)
        {
            throw std::runtime_error(
                "Failed to initialize Thruster Drive: wrong device ID: " + std::to_string(ToInt(configCtrl.Id)) +
                ", expected: " + std::to_string(ToInt(PiSubmarine::Drv8908::IcId::DRV8908)));
        }

        status = m_Chip.SetEnabledOpenLoadDetect(m_HalfBridges);
        assert(IsValid(status));
        status = m_Chip.SetOpenLoadDetectControl3(PiSubmarine::Drv8908::OcpDeglitchTime::MicroSeconds60, true);
        assert(IsValid(status));
        status = m_Chip.SetOpenLoadDetectControl2(
            PiSubmarine::Drv8908::OpenLoadDetectControl::OldRep | PiSubmarine::Drv8908::OpenLoadDetectControl::OldOp);
        assert(IsValid(status));
        PiSubmarine::Drv8908::PwmGeneratorBitMask pwmGenerators;
        status = m_Chip.GetEnabledPwmGenerators(pwmGenerators);
        assert(IsValid(status));
        pwmGenerators = pwmGenerators | PiSubmarine::Drv8908::ToPwmGeneratorBitMask(m_PwmGenerator);
        status = m_Chip.SetEnabledPwmGenerators(pwmGenerators);
        assert(IsValid(status));
        status = m_Chip.SetPwmFrequency(m_PwmGenerator, PiSubmarine::Drv8908::PwmFrequency::Hz2000);
        assert(IsValid(status));
        PiSubmarine::Drv8908::HalfBridgeBitMask pwmHalfBridges;
        status = m_Chip.GetHalfBridgePwmModes(pwmHalfBridges);
        assert(IsValid(status));
        pwmHalfBridges = pwmHalfBridges | m_HalfBridges;
        status = m_Chip.SetHalfBridgePwmModes(pwmHalfBridges);
        assert(IsValid(status));

        ApplyHalfBridgeStates();

        status = m_Chip.SetPwmMap(m_HalfBridges, m_PwmGenerator);
        assert(IsValid(status));

        status = m_Chip.SetHalfBridgeActiveFreeWheeling(m_HalfBridges);
        assert(IsValid(status));

        m_KickNeeded = true;
    }

    void ControllerBase::ReadStatus()
    {
        PiSubmarine::Drv8908::IcStatus stat;

        PiSubmarine::Drv8908::IcStatus chipStatus;
        stat = m_Chip.GetStatus(chipStatus);
        if (!PiSubmarine::Drv8908::IsValid(stat))
        {
            m_OperationalState = Telemetry::Api::OperationalState::Faulted;
            return;
        }
        m_Faults = ConvertFaults(chipStatus);
        m_Warnings = ConvertWarnings(chipStatus);
        if (m_Faults != Telemetry::Api::Faults{0} || m_Warnings != Telemetry::Api::Warnings{0})
        {
            m_OperationalState = Telemetry::Api::OperationalState::Degraded;
            PiSubmarine::Drv8908::ConfigCtrl configCtrl{};
            stat = m_Chip.GetConfigCtrl(configCtrl);
            assert(PiSubmarine::Drv8908::IsValid(stat));
            configCtrl.ClrFlt = true;
            stat = m_Chip.SetConfigCtrl(configCtrl);
            assert(PiSubmarine::Drv8908::IsValid(stat));
        }
        else
        {
            m_OperationalState = Telemetry::Api::OperationalState::Operational;
        }
    }

    void ControllerBase::TransitionDutyCycle(
        const NormalizedFraction targetDutyCycle,
        DutyRate speed,
        const std::chrono::nanoseconds deltaTime)
    {
        if (m_CurrentDutyCycle != targetDutyCycle)
        {
            double dutyDeltaCurrent = std::fabs(targetDutyCycle - m_CurrentDutyCycle);
            double dutyDeltaTick = speed * deltaTime;
            if (std::fabs(dutyDeltaTick) > dutyDeltaCurrent)
            {
                dutyDeltaTick = dutyDeltaCurrent;
            }

            if (m_CurrentDutyCycle < targetDutyCycle)
            {
                m_CurrentDutyCycle = m_CurrentDutyCycle + dutyDeltaTick;
            }
            else if (m_CurrentDutyCycle > targetDutyCycle)
            {
                m_CurrentDutyCycle = m_CurrentDutyCycle - dutyDeltaTick;
            }

            SetDutyCycleInternal(m_CurrentDutyCycle);
        }
    }

    void ControllerBase::SetDutyCycleInternal(const NormalizedFraction dutyCycle)
    {
        assert(m_PowerLease.IsValid());
        m_CurrentDutyCycle = dutyCycle;
        const NormalizedIntFraction<8> dutyCycleInt{static_cast<double>(m_CurrentDutyCycle)};
        const auto stat = m_Chip.SetDutyCycle(m_PwmGenerator, dutyCycleInt);
        assert(PiSubmarine::Drv8908::IsValid(stat));
    }

    void ControllerBase::ApplyHalfBridgeStates() const
    {
        using namespace RegUtils;

        const auto activeHalfBridges = m_HighSideHalfBridges | m_LowSideHalfBridges;
        const auto inactiveHalfBridges = static_cast<PiSubmarine::Drv8908::HalfBridgeBitMask>(
            ToInt(m_HalfBridges) & ~ToInt(activeHalfBridges));

        if (m_HighSideHalfBridges != PiSubmarine::Drv8908::HalfBridgeBitMask{0})
        {
            const auto status = m_Chip.SetHalfBridgeEnabled(m_HighSideHalfBridges, true, false);
            assert(PiSubmarine::Drv8908::IsValid(status));
        }

        if (m_LowSideHalfBridges != PiSubmarine::Drv8908::HalfBridgeBitMask{0})
        {
            const auto status = m_Chip.SetHalfBridgeEnabled(m_LowSideHalfBridges, false, true);
            assert(PiSubmarine::Drv8908::IsValid(status));
        }

        if (inactiveHalfBridges != PiSubmarine::Drv8908::HalfBridgeBitMask{0})
        {
            const auto status = m_Chip.SetHalfBridgeEnabled(inactiveHalfBridges, false, false);
            assert(PiSubmarine::Drv8908::IsValid(status));
        }
    }
}
