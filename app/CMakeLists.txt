include("${PISUBMARINE_BUILD_SYSTEM}")

project(${PISUBMARINE_MODULE_NAME}.App LANGUAGES CXX)

message("Adding executable: ${PISUBMARINE_MODULE_NAME}.App")
add_executable(${PISUBMARINE_MODULE_NAME}.App PiSubmarine/Motor/Unidirectional/Drv8908/main.cpp)
PiSubmarineInitTarget(${PISUBMARINE_MODULE_NAME}.App)

target_include_directories(${PISUBMARINE_MODULE_NAME}.App PRIVATE ./)
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE ${PISUBMARINE_MODULE_NAME})

find_package(spdlog CONFIG REQUIRED)
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE spdlog::spdlog)

find_package(ftxui CONFIG REQUIRED)
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE
        # Chose a submodule
        ftxui::component
        ftxui::dom
        ftxui::screen
)

find_package(magic_enum CONFIG REQUIRED)
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE magic_enum::magic_enum)

PiSubmarineAddDependency("https://github.com/PiSubmarine/SPI.Linux.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE PiSubmarine.SPI.Linux)

PiSubmarineAddDependency("https://github.com/PiSubmarine/GPIO.Linux.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE PiSubmarine.GPIO.Linux)