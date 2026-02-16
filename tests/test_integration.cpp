#include <gtest/gtest.h>
#include "mujin/world_model.h"

TEST(IntegrationScaffold, Placeholder) {
    mujin::WorldModel wm;
    EXPECT_EQ(wm.numFluents(), 0u);
}
