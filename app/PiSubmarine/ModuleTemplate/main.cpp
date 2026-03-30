#include <spdlog/spdlog.h>
#include "PiSubmarine/ModuleTemplate/ModuleTemplate.h"

using namespace PiSubmarine::ModuleTemplate;

int main(int argc, char* argv[])
{
    spdlog::info(PISUBMARINE_TARGET_NAME);
    ModuleTemplate moduleTemplate;
    return moduleTemplate.GetReturnCode();
}