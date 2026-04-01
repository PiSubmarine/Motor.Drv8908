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

    PiSubmarine::Motor::Unidirectional::Drv8908::Controller thrusterFrontRight(
        thrusterChip,
        thrusterChipPowerManager,
        PiSubmarine::Drv8908::PwmGenerator::PwmGenerator1,
        PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge3 | PiSubmarine::Drv8908::HalfBridgeBitMask::HalfBridge4,
        PiSubmarine::Motor::Drv8908::BridgeSide::High,
        motorConfig);

    auto screen = ScreenInteractive::TerminalOutput();

    auto render = [&]
    {
        auto operationalState = std::string(magic_enum::enum_name(thrusterFrontRight.GetOperationalState()));
        auto faults = std::string(magic_enum::enum_name(thrusterFrontRight.GetFaults()));
        auto warnings = std::string(magic_enum::enum_name(thrusterFrontRight.GetWarnings()));

        return vbox({

            text("PiSubmarine Motor Test") | bold | center,

            separator(),

            text("State: " + operationalState),
            text("Faults: " + faults),
            text("Warnings: " + warnings),

            separator(),

            text("Duty (reported): " + std::to_string(thrusterFrontRight.GetDutyCycle())),
            text("Duty (actual):  " + std::to_string(thrusterFrontRight.GetActualDutyCycle())),
            text(std::string("Power (reported): ") + (thrusterFrontRight.IsPowered() ? "ON" : "OFF")),
            text(std::string("Power (actual): ") + (thrusterFrontRight.IsActuallyPowered() ? "ON" : "OFF")),

            separator(),

            text("Controls:"),
            text("  w / s  -> duty up/down"),
            text("  p      -> power toggle"),
            text("  q      -> quit")
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

        return false;
    });

    // ---------------- Tick loop (single-threaded) ----------------
    std::thread tick_thread([&]
    {
        using namespace std::chrono;

        constexpr auto tickInterval = 10ms;
        auto nextFrameTarget = steady_clock::now() + tickInterval;
        auto lastFrameTime = steady_clock::now();

        while (true)
        {
            // 1. Calculate actual delta time since the start of the last frame
            auto now = steady_clock::now();
            auto actualDelta = duration_cast<milliseconds>(now - lastFrameTime);
            lastFrameTime = now;

            // 2. Perform work with actual delta
            thrusterFrontRight.Tick(actualDelta);
            screen.PostEvent(Event::Custom);

            // 3. Sleep until the absolute target time
            std::this_thread::sleep_until(nextFrameTarget);

            // 4. Set the next target by incrementing the previous target
            // (This maintains the 100Hz frequency even if one frame is slightly late)
            nextFrameTarget += tickInterval;
        }
    });

    screen.Loop(component);
}
