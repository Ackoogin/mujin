#include <gtest/gtest.h>
#include "mujin/world_model.h"

TEST(WorldModelScaffold, Placeholder) {
    mujin::WorldModel wm;
    EXPECT_EQ(wm.numFluents(), 0u);
    EXPECT_EQ(wm.version(), 0u);
}
