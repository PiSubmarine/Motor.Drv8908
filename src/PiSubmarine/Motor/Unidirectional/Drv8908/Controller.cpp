#include "PiSubmarine/Motor/Unidirectional/Drv8908/Controller.h"

#include <cassert>

namespace PiSubmarine::Motor::Unidirectional::Drv8908
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

    Controller::Controller(PiSubmarine::Drv8908::IDevice& chip,
                           PiSubmarine::Drv8908::IPowerManager& powerManager,
                           PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
                           PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
                           Motor::Drv8908::BridgeSide bridgeSide,
                           Motor::Drv8908::Config motorConfig) :
        m_Chip(chip),
        m_PowerManager(powerManager),
        m_PwmGenerator(pwmGenerator),
        m_HalfBridges(halfBridgeMask),
        m_BridgeSide(bridgeSide),
        m_MotorConfig(motorConfig)
    {
    }

    void Controller::SetPowered(bool enabled)
    {
        m_WantsBePowered = enabled;
    }

    bool Controller::IsPowered() const
    {
        return m_WantsBePowered;
    }

    NormalizedFraction Controller::GetDutyCycle() const
    {
        return m_TargetDutyCycle;
    }

    NormalizedFraction Controller::GetActualDutyCycle() const
    {
        return m_CurrentDutyCycle;
    }

    bool Controller::IsActuallyPowered() const
    {
        return m_PowerLease.IsValid();
    }

    void Controller::SetDutyCycle(NormalizedFraction dutyCycle)
    {
        m_TargetDutyCycle = dutyCycle;
        if (m_TargetDutyCycle < m_MotorConfig.KickDuty && m_CurrentDutyCycle < m_TargetDutyCycle)
        {
            m_KickNeeded = true;
        }
    }

    NormalizedFraction Controller::GetMinimumEffectiveDutyCycle() const
    {
        return m_MotorConfig.MinimalDuty;
    }

    void Controller::Tick(std::chrono::nanoseconds deltaTime)
    {
        PiSubmarine::Drv8908::IcStatus stat;

        if (m_WantsBePowered == false)
        {
            m_TargetDutyCycle = 0;
        }

        if (m_WantsBePowered != m_PowerLease.IsValid())
        {
            if (m_WantsBePowered == true)
            {
                PowerUp();
            }
            else if (m_CurrentDutyCycle == 0)
            {
                m_PowerManager.Release(m_PowerLease);
            }
        }

        if (m_PowerLease.IsValid())
        {
            ReadStatus();
            if (m_OperationalState == Telemetry::Api::OperationalState::Faulted)
            {
                return;
            }
        }
        else
        {
            return;
        }

        if (m_State == ControlState::Normal)
        {
            auto transitionTarget = m_TargetDutyCycle;
            if (m_TargetDutyCycle < m_MotorConfig.MinimalDuty)
            {
                transitionTarget = 0;
            }
            TransitionDutyCycle(transitionTarget, m_MotorConfig.DutyCycleChangeRate, deltaTime);

            if (m_TargetDutyCycle >= m_MotorConfig.MinimalDuty)
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
            if (m_TargetDutyCycle >= m_MotorConfig.KickDuty)
            {
                m_State = ControlState::Normal;
                m_TimeSinceKickTransition = std::chrono::nanoseconds(0);
            }
            else
            {
                TransitionDutyCycle(m_TargetDutyCycle, m_MotorConfig.KickDutyCycleChangeRate, deltaTime);

                if (m_TimeSinceKickTransition >= m_MotorConfig.KickDuration / 2)
                {
                    m_State = ControlState::Normal;
                    m_TimeSinceKickTransition = std::chrono::nanoseconds(0);
                }
            }
        }

        m_TimeSinceKickTransition += deltaTime;
    }

    Telemetry::Api::OperationalState Controller::GetOperationalState() const
    {
        return m_OperationalState;
    }

    Telemetry::Api::Faults Controller::GetFaults() const
    {
        return m_Faults;
    }

    Telemetry::Api::Warnings Controller::GetWarnings() const
    {
        return m_Warnings;
    }

    void Controller::PowerUp()
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
        status = m_Chip.SetOpenLoadDetectControl3(PiSubmarine::Drv8908::OcpDeglitchTime::MicroSeconds60, false);
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

        bool highSide = false;
        bool lowSide = false;
        if (m_BridgeSide == Motor::Drv8908::BridgeSide::High)
        {
            highSide = true;
        }
        else
        {
            lowSide = true;
        }
        status = m_Chip.SetHalfBridgeEnabled(m_HalfBridges, highSide, lowSide);
        assert(IsValid(status));
        status = m_Chip.SetPwmMap(m_HalfBridges, m_PwmGenerator);
        assert(IsValid(status));
    }

    void Controller::ReadStatus()
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

    void Controller::TransitionDutyCycle(NormalizedFraction targetDutyCycle, DutyRate speed,
                                         std::chrono::nanoseconds deltaTime)
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

    void Controller::SetDutyCycleInternal(NormalizedFraction dutyCycle)
    {
        assert(m_PowerLease.IsValid()); // We can only change Duty Cycle if we have power.
        m_CurrentDutyCycle = dutyCycle;
        NormalizedIntFraction<8> dutyCycleInt{static_cast<double>(m_CurrentDutyCycle)};
        auto stat = m_Chip.SetDutyCycle(m_PwmGenerator, dutyCycleInt);
        assert(PiSubmarine::Drv8908::IsValid(stat));
    }
}
