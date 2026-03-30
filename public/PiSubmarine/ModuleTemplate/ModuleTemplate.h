#pragma once

#include "PiSubmarine/ModuleTemplate/IModuleTemplate.h"

namespace PiSubmarine::ModuleTemplate
{
    class ModuleTemplate : public IModuleTemplate
    {
public:
        ~ModuleTemplate() override = default;
        [[nodiscard]] int GetReturnCode() const override;
    };
}

