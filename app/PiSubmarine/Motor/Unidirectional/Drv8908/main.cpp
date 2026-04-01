#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <magic_enum/magic_enum.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <PiSubmarine/Drv8908/PowerManager.h>
#include <PiSubmarine/SPI/Linux/Driver.h>
#include "PiSubmarine/Motor/Unidirectional/Drv8908/Controller.h"
#include "PiSubmarine/Motor/Unidirectional/Api/IController.h"
#include "PiSubmarine/Drv8908/Device.h"
#include "PiSubmarine/GPIO/Linux/Driver.h"
#include <spdlog/spdlog.h>

using namespace ftxui;

template <>
struct magic_enum::customize::enum_range<PiSubmarine::Motor::Telemetry::Api::Faults>
{
    static constexpr bool is_flags = true;
};

template <>
struct magic_enum::customize::enum_range<PiSubmarine::Motor::Telemetry::Api::Warnings>
{
    static constexpr bool is_flags = true;
};

void IncreaseDuty(PiSubmarine::Motor::Unidirectional::Api::IController& controller)
{
    double dutyCycle = std::min(1.0, controller.GetDutyCycle() + 0.05);
    controller.SetDutyCycle(dutyCycle);
}

void DecreaseDuty(PiSubmarine::Motor::Unidirectional::Api::IController& controller)
{
    double dutyCycle = std::max(0.0, controller.GetDutyCycle() - 0.05);
    controller.SetDutyCycle(dutyCycle);
}

// ---------------- Main ----------------
int main()
{
    using namespace PiSubmarine::RegUtils;
    using namespace std::chrono_literals;

    constexpr size_t ThrustersNSleepPin = 5;
    constexpr size_t ThrustersNFaultPin = 6;
    constexpr size_t LampsAndBallastNSleepPin = 16;
    constexpr size_t LampsAndBallastNFaultPin = 20;

    PiSubmarine::SPI::Linux::Driver thrustersSpiDriver("/dev/spidev0.0", 5000000, 8, SPI_MODE_1, SPI_MODE_1);
    PiSubmarine::SPI::Linux::Driver lampsAndBallastSpiDriver("/dev/spidev0.1", 5000000, 8,SPI_MODE_1, SPI_MODE_1);
    PiSubmarine::GPIO::Linux::Driver gpioDriver("PiSubmarine");
    auto thrustersPinGroup = gpioDriver.CreatePinGroup("Thrusters", "/dev/gpiochip0",
                                                       {ThrustersNSleepPin, ThrustersNFaultPin});
    auto lampsAndBallastPinGroup = gpioDriver.CreatePinGroup("LampsAndBallast", "/dev/gpiochip0",
                                                             {LampsAndBallastNSleepPin, LampsAndBallastNFaultPin});
    PiSubmarine::Drv8908::Device thrusterChip(thrustersSpiDriver, *thrustersPinGroup);

    PiSubmarine::Drv8908::PowerManager thrusterChipPowerManager(thrusterChip);

    PiSubmarine::Motor::Drv8908::Config motorConfig;
    motorConfig.DutyCycleChangeRate = PiSubmarine::Motor::DutyRate(1, 1s);

    // Motor 1
    PiSubmarine::Motor::Unidirectional::Drv8908::Controller thrusterFrontRight(
        thrusterChip,
        thrusterChipPowerManager,
        PiSubmarine::Drv8908::PwmGenerator::PwmGenerator1,
        PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge3 | PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge4,
        PiSubmarine::Motor::Drv8908::BridgeSide::High,
        motorConfig);

    // Motor 2
    PiSubmarine::Motor::Unidirectional::Drv8908::Controller thrusterBackRight(
        thrusterChip,
        thrusterChipPowerManager,
        PiSubmarine::Drv8908::PwmGenerator::PwmGenerator2,
        PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge7 | PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge8,
        PiSubmarine::Motor::Drv8908::BridgeSide::High,
        motorConfig);

    auto screen = ScreenInteractive::TerminalOutput();

    // Helper to render individual motor states cleanly
    auto renderMotor = [](const std::string& name, PiSubmarine::Motor::Unidirectional::Drv8908::Controller& motor)
    {
        auto operationalState = std::string(magic_enum::enum_name(motor.GetOperationalState()));
        auto faults = std::string(magic_enum::enum_name(motor.GetFaults()));
        auto warnings = std::string(magic_enum::enum_name(motor.GetWarnings()));

        return vbox({
            text(name) | bold | center,
            separator(),
            text("State: " + operationalState),
            text("Faults: " + faults),
            text("Warnings: " + warnings),
            separator(),
            text("Duty (reported): " + std::to_string(motor.GetDutyCycle())),
            text("Duty (actual):   " + std::to_string(motor.GetActualDutyCycle())),
            text(std::string("Power (reported): ") + (motor.IsPowered() ? "ON" : "OFF")),
            text(std::string("Power (actual):   ") + (motor.IsActuallyPowered() ? "ON" : "OFF")),
        });
    };

    auto render = [&]
    {
        return vbox({
            text("PiSubmarine Dual Motor Test") | bold | center,

            separator(),

            // Display the two motors side-by-side
            hbox({
                renderMotor("Front Right (Motor 1)", thrusterFrontRight) | flex,
                separator(),
                renderMotor("Back Right (Motor 2)", thrusterBackRight) | flex,
            }),

            separator(),

            text("Controls:"),
            text("  [w / s] -> Front Right duty up/down"),
            text("  [p]     -> Front Right power toggle"),
            text("  [i / k] -> Back Right duty up/down"),
            text("  [o]     -> Back Right power toggle"),
            text("  [q]     -> quit")
        }) | border;
    };

    auto component = Renderer([&]
    {
        return render();
    });

    // ---------------- Key handling ----------------
    component |= CatchEvent([&](Event e)
    {
        if (e == Event::Character('q'))
        {
            screen.ExitLoopClosure()();
            return true;
        }

        // --- Front Right Controls ---
        if (e == Event::Character('p'))
        {
            thrusterFrontRight.SetPowered(!thrusterFrontRight.IsPowered());
            return true;
        }
        if (e == Event::Character('w'))
        {
            IncreaseDuty(thrusterFrontRight);
            return true;
        }
        if (e == Event::Character('s'))
        {
            DecreaseDuty(thrusterFrontRight);
            return true;
        }

        // --- Back Right Controls ---
        if (e == Event::Character('o'))
        {
            thrusterBackRight.SetPowered(!thrusterBackRight.IsPowered());
            return true;
        }
        if (e == Event::Character('i'))
        {
            IncreaseDuty(thrusterBackRight);
            return true;
        }
        if (e == Event::Character('k'))
        {
            DecreaseDuty(thrusterBackRight);
            return true;
        }

        return false;
    });

    // ---------------- Tick loop (single-threaded) ----------------
    std::atomic<bool> isRunning{true};
    std::thread tick_thread([&]
    {
        using namespace std::chrono;

        constexpr auto tickInterval = 10ms;
        auto nextFrameTarget = steady_clock::now() + tickInterval;
        auto lastFrameTime = steady_clock::now();

        while (isRunning)
        {
            auto now = steady_clock::now();
            auto actualDelta = duration_cast<milliseconds>(now - lastFrameTime);
            lastFrameTime = now;

            // Tick both motors
            thrusterFrontRight.Tick(actualDelta);
            thrusterBackRight.Tick(actualDelta);

            screen.PostEvent(Event::Custom);

            std::this_thread::sleep_until(nextFrameTarget);
            nextFrameTarget += tickInterval;
        }
    });

    screen.Loop(component);

    // Ensure the background thread shuts down gracefully when we quit
    isRunning = false;
    if (tick_thread.joinable()) {
        tick_thread.join();
    }

    return 0;
}