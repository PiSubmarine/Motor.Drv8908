#include <gtest/gtest.h>
#include "PiSubmarine/ModuleTemplate/ModuleTemplate.h"

namespace PiSubmarine::ModuleTemplate
{
    TEST(ModuleTemplateTest, GetReturnCode)
    {
        ModuleTemplate moduleTemplate;
        ASSERT_EQ(moduleTemplate.GetReturnCode(), 0);
    }
}