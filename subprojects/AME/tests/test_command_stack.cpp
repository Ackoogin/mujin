#include <gtest/gtest.h>
#include "command_stack.h"

TEST(CommandStack, AddThenUndo) {
  ProjectModel m;
  CommandStack stack;
  stack.execute(m, "add a", [](ProjectModel& mm) {
    mm.predicates.push_back({"at", {}, 0, 0});
  });
  EXPECT_EQ(m.predicates.size(), 1u);
  EXPECT_TRUE(stack.undo(m));
  EXPECT_EQ(m.predicates.size(), 0u);
}

TEST(CommandStack, UndoThenRedo) {
  ProjectModel m;
  CommandStack stack;
  stack.execute(m, "add", [](ProjectModel& mm) {
    mm.predicates.push_back({"p", {}, 0, 0});
  });
  ASSERT_TRUE(stack.undo(m));
  ASSERT_TRUE(stack.redo(m));
  EXPECT_EQ(m.predicates.size(), 1u);
}

TEST(CommandStack, DepthLimit) {
  ProjectModel m;
  CommandStack stack(3);
  for (int i = 0; i < 5; ++i) {
    stack.execute(m, "add", [](ProjectModel& mm) {
      mm.predicates.push_back({"p", {}, 0, 0});
    });
  }
  // Only the most recent 3 entries are retained.
  size_t undoCount = 0;
  while (stack.canUndo()) {
    stack.undo(m);
    ++undoCount;
  }
  EXPECT_EQ(undoCount, 3u);
}

TEST(CommandStack, RedoStackClearedOnNewExecute) {
  ProjectModel m;
  CommandStack stack;
  stack.execute(m, "a", [](ProjectModel& mm) { mm.predicates.push_back({"a", {}, 0, 0}); });
  stack.undo(m);
  EXPECT_TRUE(stack.canRedo());
  stack.execute(m, "b", [](ProjectModel& mm) { mm.predicates.push_back({"b", {}, 0, 0}); });
  EXPECT_FALSE(stack.canRedo());
}
