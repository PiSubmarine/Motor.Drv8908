#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "PiSubmarine/Drv8908/IDriverMock.h"
#include "PiSubmarine/Drv8908/IPowerManager.h"
#include "PiSubmarine/Motor/Bidirectional/Drv8908/Controller.h"
#include "PiSubmarine/Motor/Unidirectional/Drv8908/Controller.h"
#include "PiSubmarine/RegUtils.h"

namespace PiSubmarine::Motor::Drv8908
{
    namespace
    {
        using namespace PiSubmarine::RegUtils;

        class TestPowerManager : public PiSubmarine::Drv8908::IPowerManager
        {
        public:
            PiSubmarine::Drv8908::PowerLease Acquire() override
            {
                ++AcquireCount;
                return CreateLease(this, AcquireCount);
            }

            void Release(PiSubmarine::Drv8908::PowerLease& lease) override
            {
                ++ReleaseCount;
                SetLeaseManager(lease, nullptr);
                SetLeaseUserIndex(lease, -1);
            }

            int AcquireCount = 0;
            int ReleaseCount = 0;
        };

        [[nodiscard]] constexpr PiSubmarine::Drv8908::IcStatus ValidSpiStatus()
        {
            return PiSubmarine::Drv8908::IcStatus::TestBit;
        }

        void PrepareSuccessfulConfigurationDefaults(testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock>& chip)
        {
            ON_CALL(chip, GetStatus(testing::_))
                .WillByDefault([](PiSubmarine::Drv8908::IcStatus& icStatus)
                {
                    icStatus = static_cast<PiSubmarine::Drv8908::IcStatus>(0);
                    return ValidSpiStatus();
                });
            ON_CALL(chip, GetConfigCtrl(testing::_))
                .WillByDefault([](PiSubmarine::Drv8908::ConfigCtrl& configCtrl)
                {
                    configCtrl = {
                        .PoldEn = false,
                        .Id = PiSubmarine::Drv8908::IcId::DRV8908,
                        .OcpRep = false,
                        .OtwRep = false,
                        .ExtOvp = false,
                        .ClrFlt = false
                    };
                    return ValidSpiStatus();
                });
            ON_CALL(chip, SetConfigCtrl(testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, SetEnabledOpenLoadDetect(testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, SetOpenLoadDetectControl3(testing::_, testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, SetOpenLoadDetectControl2(testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, GetEnabledPwmGenerators(testing::_))
                .WillByDefault([](PiSubmarine::Drv8908::PwmGeneratorBitMask& generators)
                {
                    generators = static_cast<PiSubmarine::Drv8908::PwmGeneratorBitMask>(0);
                    return ValidSpiStatus();
                });
            ON_CALL(chip, SetEnabledPwmGenerators(testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, SetPwmFrequency(testing::A<PiSubmarine::Drv8908::PwmGenerator>(), testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, GetHalfBridgePwmModes(testing::_))
                .WillByDefault([](PiSubmarine::Drv8908::HalfBridgeBitMask& channels)
                {
                    channels = static_cast<PiSubmarine::Drv8908::HalfBridgeBitMask>(0);
                    return ValidSpiStatus();
                });
            ON_CALL(chip, SetHalfBridgePwmModes(testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, SetHalfBridgeEnabled(testing::A<PiSubmarine::Drv8908::HalfBridgeBitMask>(), testing::_, testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, SetPwmMap(testing::A<PiSubmarine::Drv8908::HalfBridgeBitMask>(), testing::A<PiSubmarine::Drv8908::PwmGenerator>()))
                .WillByDefault(testing::Return(ValidSpiStatus()));
            ON_CALL(chip, SetDutyCycle(testing::A<PiSubmarine::Drv8908::PwmGenerator>(), testing::_))
                .WillByDefault(testing::Return(ValidSpiStatus()));
        }
    }

    TEST(UnidirectionalControllerTest, ReportsForwardTelemetryWhenRunning)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        Unidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator1,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge1 | PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge2,
            Motor::Drv8908::BridgeSide::High,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(NormalizedFraction{0.6}).has_value());

        controller.Tick(0ns, 10ms);

        const auto state = controller.GetState();
        ASSERT_TRUE(state.has_value());
        EXPECT_EQ(state->Direction, Telemetry::Api::DriveDirection::Forward);
        EXPECT_DOUBLE_EQ(static_cast<double>(state->DriveEffort), 0.6);
        EXPECT_EQ(powerManager.AcquireCount, 1);
    }

    TEST(UnidirectionalControllerTest, ReleasesPowerAfterRampingToZeroWhenTurnedOff)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        Unidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator2,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge3,
            Motor::Drv8908::BridgeSide::Low,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(NormalizedFraction{0.4}).has_value());
        controller.Tick(0ns, 10ms);

        ASSERT_TRUE(controller.SetPowered(false).has_value());
        controller.Tick(10ms, 10ms);
        controller.Tick(20ms, 10ms);

        EXPECT_EQ(powerManager.AcquireCount, 1);
        EXPECT_GE(powerManager.ReleaseCount, 1);
        EXPECT_DOUBLE_EQ(static_cast<double>(controller.GetActualDutyCycle().value()), 0.0);
    }

    TEST(UnidirectionalControllerTest, CoastsBelowMinimalDutyUntilDriveThresholdIsReached)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        Unidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator2,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge3,
            Motor::Drv8908::BridgeSide::Low,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(NormalizedFraction{0.19}).has_value());

        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge3,
            false,
            false))
            .Times(testing::AtLeast(1));
        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge3,
            false,
            true))
            .Times(0);

        controller.Tick(0ns, 10ms);

        EXPECT_DOUBLE_EQ(static_cast<double>(controller.GetActualDutyCycle().value()), 0.0);

        testing::Mock::VerifyAndClearExpectations(&chip);

        ASSERT_TRUE(controller.SetDutyCycle(NormalizedFraction{0.20}).has_value());

        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge3,
            false,
            true))
            .Times(testing::AtLeast(1));

        controller.Tick(10ms, 10ms);

        EXPECT_DOUBLE_EQ(static_cast<double>(controller.GetActualDutyCycle().value()), 0.20);
    }

    TEST(BidirectionalControllerTest, UsesConfiguredHalfBridgeMasksForForwardAndReverse)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        Bidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator4,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(SignedNormalizedFraction{0.3}).has_value());

        EXPECT_CALL(
            chip,
            SetHalfBridgeEnabled(
                PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7,
                true,
                false))
            .Times(testing::AtLeast(1));
        EXPECT_CALL(
            chip,
            SetHalfBridgeEnabled(
                PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
                false,
                true))
            .Times(testing::AtLeast(1));
        EXPECT_CALL(chip, SetDutyCycle(PiSubmarine::Drv8908::PwmGenerator::PwmGenerator4, testing::_))
            .Times(testing::AtLeast(1));

        controller.Tick(0ns, 10ms);

        ASSERT_TRUE(controller.SetDutyCycle(SignedNormalizedFraction{-0.3}).has_value());

        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7,
            false,
            true));
        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
            true,
            false));

        controller.Tick(10ms, 10ms);
        controller.Tick(20ms, 10ms);
    }

    TEST(BidirectionalControllerTest, ReportsReverseTelemetryDirection)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        Bidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator5,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge6,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge5,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(SignedNormalizedFraction{-0.7}).has_value());

        controller.Tick(0ns, 10ms);

        const auto state = controller.GetState();
        ASSERT_TRUE(state.has_value());
        EXPECT_EQ(state->Direction, Telemetry::Api::DriveDirection::Reverse);
        EXPECT_DOUBLE_EQ(static_cast<double>(state->DriveEffort), 0.7);
    }

    TEST(BidirectionalControllerTest, CoastsBelowMinimalDutyAndRestoresDirectionWhenDriving)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        Bidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator4,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(SignedNormalizedFraction{-0.19}).has_value());

        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7 | PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
            false,
            false))
            .Times(testing::AtLeast(1));
        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7,
            false,
            true))
            .Times(0);
        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
            true,
            false))
            .Times(0);

        controller.Tick(0ns, 10ms);

        const auto coastState = controller.GetState();
        ASSERT_TRUE(coastState.has_value());
        EXPECT_EQ(coastState->Direction, Telemetry::Api::DriveDirection::Idle);
        EXPECT_DOUBLE_EQ(static_cast<double>(coastState->DriveEffort), 0.0);

        testing::Mock::VerifyAndClearExpectations(&chip);

        ASSERT_TRUE(controller.SetDutyCycle(SignedNormalizedFraction{-0.20}).has_value());

        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7,
            false,
            true))
            .Times(testing::AtLeast(1));
        EXPECT_CALL(chip, SetHalfBridgeEnabled(
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
            true,
            false))
            .Times(testing::AtLeast(1));

        controller.Tick(10ms, 10ms);

        const auto driveState = controller.GetState();
        ASSERT_TRUE(driveState.has_value());
        EXPECT_EQ(driveState->Direction, Telemetry::Api::DriveDirection::Reverse);
        EXPECT_DOUBLE_EQ(static_cast<double>(driveState->DriveEffort), 0.20);
    }

    TEST(UnidirectionalControllerTest, RetriesPowerUpAfterSpiFailureOnNextTick)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        EXPECT_CALL(chip, GetStatus(testing::_))
            .WillOnce([](PiSubmarine::Drv8908::IcStatus&)
            {
                return PiSubmarine::Drv8908::IcStatus{0};
            })
            .WillRepeatedly([](PiSubmarine::Drv8908::IcStatus& icStatus)
            {
                icStatus = static_cast<PiSubmarine::Drv8908::IcStatus>(0);
                return ValidSpiStatus();
            });

        Unidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator1,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge1,
            Motor::Drv8908::BridgeSide::High,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(NormalizedFraction{0.4}).has_value());

        controller.Tick(0ns, 10ms);

        EXPECT_FALSE(controller.IsActuallyPowered());
        EXPECT_EQ(powerManager.AcquireCount, 1);
        EXPECT_EQ(powerManager.ReleaseCount, 1);
        const auto failedState = controller.GetState();
        ASSERT_TRUE(failedState.has_value());
        EXPECT_EQ(failedState->Operational, Telemetry::Api::OperationalState::Faulted);

        controller.Tick(10ms, 10ms);

        EXPECT_TRUE(controller.IsActuallyPowered());
        EXPECT_EQ(powerManager.AcquireCount, 2);
        EXPECT_EQ(powerManager.ReleaseCount, 1);
    }

    TEST(UnidirectionalControllerTest, DoesNotThrowAndRetriesAfterUnexpectedDeviceId)
    {
        using namespace std::chrono_literals;

        testing::NiceMock<PiSubmarine::Drv8908::IDeviceMock> chip;
        TestPowerManager powerManager;
        PrepareSuccessfulConfigurationDefaults(chip);

        EXPECT_CALL(chip, GetConfigCtrl(testing::_))
            .WillOnce([](PiSubmarine::Drv8908::ConfigCtrl& configCtrl)
            {
                configCtrl = {
                    .PoldEn = false,
                    .Id = PiSubmarine::Drv8908::IcId::DRV8906,
                    .OcpRep = false,
                    .OtwRep = false,
                    .ExtOvp = false,
                    .ClrFlt = false
                };
                return ValidSpiStatus();
            })
            .WillRepeatedly([](PiSubmarine::Drv8908::ConfigCtrl& configCtrl)
            {
                configCtrl = {
                    .PoldEn = false,
                    .Id = PiSubmarine::Drv8908::IcId::DRV8908,
                    .OcpRep = false,
                    .OtwRep = false,
                    .ExtOvp = false,
                    .ClrFlt = false
                };
                return ValidSpiStatus();
            });

        Unidirectional::Drv8908::Controller controller(
            chip,
            powerManager,
            PiSubmarine::Drv8908::PwmGenerator::PwmGenerator2,
            PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge2,
            Motor::Drv8908::BridgeSide::High,
            Motor::Drv8908::Config{
                .DutyCycleChangeRate = DutyRate{1, 10ms},
                .MinimalDuty = NormalizedFraction{0.20},
                .KickDuration = 0ms,
                .KickInterval = 0ms,
                .KickDuty = NormalizedFraction{0.50},
                .KickDutyCycleChangeRate = DutyRate{1, 1ms}});

        ASSERT_TRUE(controller.SetPowered(true).has_value());
        ASSERT_TRUE(controller.SetDutyCycle(NormalizedFraction{0.4}).has_value());

        EXPECT_NO_THROW(controller.Tick(0ns, 10ms));
        EXPECT_FALSE(controller.IsActuallyPowered());
        EXPECT_EQ(powerManager.AcquireCount, 1);
        EXPECT_EQ(powerManager.ReleaseCount, 1);

        controller.Tick(10ms, 10ms);

        EXPECT_TRUE(controller.IsActuallyPowered());
        EXPECT_EQ(powerManager.AcquireCount, 2);
        EXPECT_EQ(powerManager.ReleaseCount, 1);
    }
}
