include("${PISUBMARINE_BUILD_SYSTEM}")

message("Adding library: ${PISUBMARINE_MODULE_NAME}")
add_library(${PISUBMARINE_MODULE_NAME}
        PiSubmarine/Motor/Unidirectional/Drv8908/Controller.cpp)

target_include_directories(${PISUBMARINE_MODULE_NAME} PUBLIC "../public")
PiSubmarineInitTarget(${PISUBMARINE_MODULE_NAME})

PiSubmarineAddDependency("https://github.com/PiSubmarine/Drv8908.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PiSubmarine.Drv8908)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Drv8908.PowerManager.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PiSubmarine.Drv8908.PowerManager)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Motor.Unidirectional.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PiSubmarine.Motor.Unidirectional.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Motor.Bidirectional.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PiSubmarine.Motor.Bidirectional.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Motor.Telemetry.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PiSubmarine.Motor.Telemetry.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Time.Tickable.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PiSubmarine.Time.Tickable.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Motor.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PiSubmarine.Motor)

