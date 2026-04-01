#pragma once

#include "PiSubmarine/Drv8908/IDevice.h"
#include "PiSubmarine/Drv8908/IPowerManager.h"
#include "PiSubmarine/Drv8908/PwmGenerator.h"
#include "PiSubmarine/Drv8908/HalfBridgeBitMask.h"
#include "PiSubmarine/Motor/Unidirectional/Api/IController.h"
#include "PiSubmarine/Motor/Drv8908/BridgeSide.h"
#include "PiSubmarine/Time/ITickable.h"
#include "PiSubmarine/Motor/DutyRate.h"
#include "PiSubmarine/Motor/Telemetry/Api/IProvider.h"
#include "PiSubmarine/Motor/Unidirectional/Drv8908/Config.h"


namespace PiSubmarine::Motor::Unidirectional::Drv8908
{
    class Controller : public Motor::Unidirectional::Api::IController, Telemetry::Api::IProvider, Time::ITickable
    {
    public:
        enum class ControlState
        {
            Normal = 0,
            KickRise,
            KickFall
        };

        Controller(
            PiSubmarine::Drv8908::IDevice& chip,
            PiSubmarine::Drv8908::IPowerManager& powerManager,
            PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
            PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
            Motor::Drv8908::BridgeSide bridgeSide,
            Motor::Drv8908::Config motorConfig
        );

        void SetPowered(bool enabled) override;
        [[nodiscard]] bool IsPowered() const override;
        [[nodiscard]] NormalizedFraction GetDutyCycle() const override;
        [[nodiscard]] NormalizedFraction GetActualDutyCycle() const;
        [[nodiscard]] bool IsActuallyPowered() const;
        void SetDutyCycle(NormalizedFraction dutyCycle) override;
        NormalizedFraction GetMinimumEffectiveDutyCycle() const override;

        void Tick(std::chrono::nanoseconds deltaTime) override;

        [[nodiscard]] Telemetry::Api::OperationalState GetOperationalState() const override;
        [[nodiscard]] Telemetry::Api::Faults GetFaults() const override;
        [[nodiscard]] Telemetry::Api::Warnings GetWarnings() const override;

    private:
        PiSubmarine::Drv8908::IDevice& m_Chip;
        PiSubmarine::Drv8908::IPowerManager& m_PowerManager;
        PiSubmarine::Drv8908::PowerLease m_PowerLease;

        PiSubmarine::Drv8908::PwmGenerator m_PwmGenerator;
        PiSubmarine::Drv8908::HalfBridgeBitMask m_HalfBridges;
        Motor::Drv8908::BridgeSide m_BridgeSide;
        Motor::Drv8908::Config m_MotorConfig;

        ControlState m_State = ControlState::Normal;
        std::chrono::nanoseconds m_TimeSinceKickTransition;
        bool m_WantsBePowered = false;
        bool m_KickNeeded = true;
        NormalizedFraction m_CurrentDutyCycle{0};
        NormalizedFraction m_TargetDutyCycle{0};

        Telemetry::Api::OperationalState m_OperationalState{Telemetry::Api::OperationalState::Operational};
        Telemetry::Api::Faults m_Faults{0};
        Telemetry::Api::Warnings m_Warnings{0};

        void PowerUp();
        void ReadStatus();
        void TransitionDutyCycle(NormalizedFraction targetDutyCycle, DutyRate speed, std::chrono::nanoseconds deltaTime);
        void SetDutyCycleInternal(NormalizedFraction dutyCycle);
    };
}
