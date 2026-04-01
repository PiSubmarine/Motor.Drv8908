#pragma once

#include <gmock/gmock.h>
#include "PiSubmarine/Drv8908/IModuleTemplate.h"

namespace PiSubmarine::ModuleTemplate
{
    class IModuleTemplateMock : public IModuleTemplate
    {
    public:
        MOCK_METHOD(int, GetReturnCode, (), (const, override));
    };
}
