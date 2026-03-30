include("${PISUBMARINE_BUILD_SYSTEM}")

project(${PISUBMARINE_MODULE_NAME}.App LANGUAGES CXX)

message("Adding executable: ${PISUBMARINE_MODULE_NAME}.App")
add_executable(${PISUBMARINE_MODULE_NAME}.App PiSubmarine/ModuleTemplate/main.cpp)
PiSubmarineInitTarget(${PISUBMARINE_MODULE_NAME}.App)

target_include_directories(${PISUBMARINE_MODULE_NAME}.App PRIVATE ./)
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE ${PISUBMARINE_MODULE_NAME})

find_package(spdlog CONFIG REQUIRED)
target_link_libraries(${PISUBMARINE_MODULE_NAME}.App PRIVATE spdlog::spdlog)