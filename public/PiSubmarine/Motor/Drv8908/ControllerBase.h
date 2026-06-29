#pragma once

#include <chrono>

#include "PiSubmarine/Drv8908/HalfBridgeBitMask.h"
#include "PiSubmarine/Drv8908/IDevice.h"
#include "PiSubmarine/Drv8908/IPowerManager.h"
#include "PiSubmarine/Drv8908/PwmGenerator.h"
#include "PiSubmarine/Motor/Drv8908/BridgeSide.h"
#include "PiSubmarine/Motor/Drv8908/Config.h"
#include "PiSubmarine/Motor/Telemetry/Api/IProvider.h"
#include "PiSubmarine/Time/ITickable.h"

namespace PiSubmarine::Motor::Drv8908
{
    class ControllerBase : public Telemetry::Api::IProvider, public Time::ITickable
    {
    public:
        enum class ControlState
        {
            Normal = 0,
            KickRise,
            KickFall
        };

        ControllerBase(
            PiSubmarine::Drv8908::IDevice& chip,
            PiSubmarine::Drv8908::IPowerManager& powerManager,
            PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
            PiSubmarine::Drv8908::HalfBridgeBitMask halfBridgeMask,
            BridgeSide initialBridgeSide,
            Config motorConfig);

        ControllerBase(
            PiSubmarine::Drv8908::IDevice& chip,
            PiSubmarine::Drv8908::IPowerManager& powerManager,
            PiSubmarine::Drv8908::PwmGenerator pwmGenerator,
            PiSubmarine::Drv8908::HalfBridgeBitMask initialHighSideHalfBridgeMask,
            PiSubmarine::Drv8908::HalfBridgeBitMask initialLowSideHalfBridgeMask,
            Config motorConfig);

        [[nodiscard]] virtual Error::Api::Result<void> SetPowered(bool enabled);
        [[nodiscard]] virtual Error::Api::Result<bool> IsPowered() const;
        [[nodiscard]] virtual Error::Api::Result<NormalizedFraction> GetActualDutyCycle() const;
        [[nodiscard]] virtual bool IsActuallyPowered() const;
        [[nodiscard]] virtual Error::Api::Result<NormalizedFraction> GetMinimumEffectiveDutyCycle() const;

    protected:
        [[nodiscard]] bool WantsBePowered() const;
        void ClampTargetDutyCycleIfUnpowered(NormalizedFraction& targetDutyCycle) const;
        void NoteTargetDutyCycleChange(NormalizedFraction oldDutyCycle, NormalizedFraction newDutyCycle);
        [[nodiscard]] bool BeginTick();
        void TickDriveEffort(NormalizedFraction targetDutyCycle, const std::chrono::nanoseconds& deltaTime);
        [[nodiscard]] Error::Api::Result<void> SetBridgeSide(BridgeSide bridgeSide);
        [[nodiscard]] Error::Api::Result<void> SetHalfBridgeStates(
            PiSubmarine::Drv8908::HalfBridgeBitMask highSideHalfBridgeMask,
            PiSubmarine::Drv8908::HalfBridgeBitMask lowSideHalfBridgeMask);
        [[nodiscard]] Error::Api::Result<Telemetry::Api::State> GetStateForDirection(
            Telemetry::Api::DriveDirection direction) const;
        void RequestKick();

    private:
        PiSubmarine::Drv8908::IDevice& m_Chip;
        PiSubmarine::Drv8908::IPowerManager& m_PowerManager;
        PiSubmarine::Drv8908::PowerLease m_PowerLease;

        PiSubmarine::Drv8908::PwmGenerator m_PwmGenerator;
        PiSubmarine::Drv8908::HalfBridgeBitMask m_HalfBridges;
        PiSubmarine::Drv8908::HalfBridgeBitMask m_RequestedHighSideHalfBridges;
        PiSubmarine::Drv8908::HalfBridgeBitMask m_RequestedLowSideHalfBridges;
        Config m_MotorConfig;

        ControlState m_State = ControlState::Normal;
        std::chrono::nanoseconds m_TimeSinceKickTransition{};
        bool m_WantsBePowered = false;
        bool m_KickNeeded = true;
        NormalizedFraction m_CurrentDutyCycle{0};

        Telemetry::Api::OperationalState m_OperationalState{Telemetry::Api::OperationalState::Operational};
        Telemetry::Api::Faults m_Faults{0};
        Telemetry::Api::Warnings m_Warnings{0};

        [[nodiscard]] Error::Api::Result<void> PowerUp();
        [[nodiscard]] Error::Api::Result<void> ReadStatus();
        [[nodiscard]] Error::Api::Result<void> TransitionDutyCycle(
            NormalizedFraction targetDutyCycle,
            DutyRate speed,
            std::chrono::nanoseconds deltaTime);
        [[nodiscard]] Error::Api::Result<void> SetDutyCycleInternal(NormalizedFraction dutyCycle);
        [[nodiscard]] bool ShouldDrive(NormalizedFraction dutyCycle) const;
        [[nodiscard]] Error::Api::Result<void> ApplyHalfBridgeStates(NormalizedFraction dutyCycle) const;
    };
}
