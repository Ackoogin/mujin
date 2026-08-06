#pragma once

#include <string>
#include <vector>

/// Finds clickable items that share an identity.
///
/// Dear ImGui works out what to call an item from its label and from the
/// identity stack in force at the time. Two items that end up with the same
/// identity become, as far as the library is concerned, one item: clicking one
/// activates the other, and typing in one can change the other. It is an easy
/// mistake to make when a loop draws several rows whose controls are named for
/// their position rather than their contents.
///
/// Dear ImGui does notice this by itself, but only while the mouse happens to
/// be resting on one of the items involved. That never happens in an automated
/// run with no one at the screen, so its own report cannot be used as a test.
///
/// This uses the hook Dear ImGui provides for its test framework, which is
/// called as every item is submitted, and simply looks for repeats. It sees the
/// whole interface rather than one panel, so a fault of this kind is reported
/// wherever it appears.
namespace imgui_id_audit {

/// A single identity that was used more than once in one frame.
struct Duplicate {
  std::string window;   ///< Window the repeated item was drawn in.
  unsigned int id = 0;  ///< The identity that was reused.
  int count = 0;        ///< How many items claimed it.
};

/// Starts watching. Call once, after the Dear ImGui context exists.
void enable();

/// Clears what was recorded. Call immediately before ImGui::NewFrame().
void beginFrame();

/// Identities used by more than one item during the frame just drawn.
const std::vector<Duplicate>& duplicates();

/// A readable description of the first repeat, for a test failure message.
std::string describeFirstDuplicate();

}  // namespace imgui_id_audit
