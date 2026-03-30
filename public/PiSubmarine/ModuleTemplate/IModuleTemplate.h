#pragma once

namespace PiSubmarine::ModuleTemplate
{
    class IModuleTemplate
    {
public:
        virtual ~IModuleTemplate() = default;
        [[nodiscard]] virtual int GetReturnCode() const = 0;
    };
}

