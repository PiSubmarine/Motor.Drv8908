#include <gtest/gtest.h>
#include "PiSubmarine/Drv8908/ModuleTemplate.h"

namespace PiSubmarine::ModuleTemplate
{
    TEST(ModuleTemplateTest, GetReturnCode)
    {
        ModuleTemplate moduleTemplate;
        ASSERT_EQ(moduleTemplate.GetReturnCode(), 0);
    }
}