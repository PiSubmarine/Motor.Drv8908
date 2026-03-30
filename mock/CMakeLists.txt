include("${PISUBMARINE_BUILD_SYSTEM}")

add_library(${PISUBMARINE_MODULE_NAME}.Mock INTERFACE PiSubmarine/ModuleTemplate/IModuleTemplateMock.h)
target_include_directories(${PISUBMARINE_MODULE_NAME}.Mock INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
PiSubmarineInitTarget(${PISUBMARINE_MODULE_NAME}.Mock)

find_package(GTest)
target_link_libraries(${PISUBMARINE_MODULE_NAME}.Mock INTERFACE GTest::gtest_main GTest::gmock_main)