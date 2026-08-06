#include "imgui_id_audit.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <algorithm>
#include <cstdarg>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct Seen {
  int count = 0;
  std::string window;
};

std::unordered_map<unsigned int, Seen>& seenThisFrame() {
  static std::unordered_map<unsigned int, Seen> seen;
  return seen;
}

std::vector<imgui_id_audit::Duplicate>& duplicateList() {
  static std::vector<imgui_id_audit::Duplicate> duplicates;
  return duplicates;
}

bool g_enabled = false;

}  // namespace

namespace imgui_id_audit {

void enable() {
  ImGuiContext* ctx = ImGui::GetCurrentContext();
  if (ctx == nullptr) {
    return;
  }
  // Turns on the per-item hook below. Left off in normal use so that a build
  // nobody is testing does no extra work.
  ctx->TestEngineHookItems = true;
  g_enabled = true;
}

void beginFrame() {
  seenThisFrame().clear();
  duplicateList().clear();
}

const std::vector<Duplicate>& duplicates() { return duplicateList(); }

std::string describeFirstDuplicate() {
  const std::vector<Duplicate>& found = duplicateList();
  if (found.empty()) {
    return {};
  }
  const Duplicate& first = found.front();
  return std::to_string(found.size()) + " identity/identities were reused; " +
         "the first was used by " + std::to_string(first.count) +
         " items in window '" + first.window +
         "'. Separate them with PushID or a ##suffix.";
}

}  // namespace imgui_id_audit

// ---------------------------------------------------------------------------
// The hooks Dear ImGui expects when IMGUI_ENABLE_TEST_ENGINE is defined. It
// declares these and calls them; something has to define them, and normally
// that is the separate test-engine library. Only the first is of interest here.
// ---------------------------------------------------------------------------

void ImGuiTestEngineHook_ItemAdd(ImGuiContext* ctx,
                                 ImGuiID id,
                                 const ImRect& /*bb*/,
                                 const ImGuiLastItemData* item_data) {
  if (!g_enabled || ctx == nullptr || id == 0) {
    return;
  }
  // Some items are allowed to repeat an identity on purpose, and Dear ImGui
  // marks those. Anything else that repeats is a mistake.
  if (item_data != nullptr &&
      (item_data->ItemFlags & ImGuiItemFlags_AllowDuplicateId) != 0) {
    return;
  }

  const char* window_name = ctx->CurrentWindow != nullptr &&
                                    ctx->CurrentWindow->Name != nullptr
                                ? ctx->CurrentWindow->Name
                                : "(unknown window)";

  Seen& entry = seenThisFrame()[static_cast<unsigned int>(id)];
  ++entry.count;
  if (entry.count == 1) {
    entry.window = window_name;
    return;
  }
  if (entry.count == 2) {
    duplicateList().push_back(
        {entry.window, static_cast<unsigned int>(id), entry.count});
    return;
  }
  for (imgui_id_audit::Duplicate& duplicate : duplicateList()) {
    if (duplicate.id == static_cast<unsigned int>(id)) {
      duplicate.count = entry.count;
      return;
    }
  }
}

void ImGuiTestEngineHook_ItemInfo(ImGuiContext* /*ctx*/,
                                  ImGuiID /*id*/,
                                  const char* /*label*/,
                                  ImGuiItemStatusFlags /*flags*/) {}

void ImGuiTestEngineHook_Log(ImGuiContext* /*ctx*/, const char* /*fmt*/, ...) {}

const char* ImGuiTestEngine_FindItemDebugLabel(ImGuiContext* /*ctx*/,
                                               ImGuiID /*id*/) {
  return nullptr;
}
