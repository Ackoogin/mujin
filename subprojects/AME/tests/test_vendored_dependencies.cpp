// Checks the third-party sources checked in under subprojects/AME/external.
//
// AME is deployed into air-gapped environments, so those copies are what a
// deployment build compiles: nothing is downloaded. That makes them part of the
// source tree rather than a cache, and this file guards them the way any other
// part of the tree is guarded.
//
// What it checks is deliberately cheap: that every dependency the build expects
// is present, that the manifest describing them agrees with what is on disk,
// and that no upstream version-control directory came along with the copy. It
// does not hash the trees; scripts/vendor_dependencies.py --verify does that,
// and it takes long enough that it belongs in continuous integration rather
// than in a test that runs on every build.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>

namespace fs = std::filesystem;

namespace {

fs::path externalDir() {
  return fs::path(AME_EXTERNAL_DIR);
}

nlohmann::json readManifest() {
  std::ifstream stream(externalDir() / "manifest.json");
  EXPECT_TRUE(stream.good()) << "external/manifest.json is missing";
  nlohmann::json manifest;
  stream >> manifest;
  return manifest;
}

}  // namespace

TEST(VendoredDependencies, ManifestListsEveryCheckedInCopy) {
  const nlohmann::json manifest = readManifest();
  ASSERT_TRUE(manifest.contains("dependencies"));

  std::set<std::string> recorded;
  for (const auto& entry : manifest["dependencies"]) {
    recorded.insert(entry.at("name").get<std::string>());
  }

  std::set<std::string> onDisk;
  for (const auto& entry : fs::directory_iterator(externalDir())) {
    if (entry.is_directory()) {
      onDisk.insert(entry.path().filename().string());
    }
  }

  EXPECT_EQ(recorded, onDisk)
      << "external/manifest.json and external/ disagree about which "
         "dependencies are checked in. Re-run "
         "subprojects/AME/scripts/vendor_dependencies.py.";
}

TEST(VendoredDependencies, EveryDependencyTheBuildNeedsIsPresent) {
  // The names the CMake module looks for. A dependency added to the build
  // without being vendored would leave an air-gapped build reaching for the
  // network, which is the failure this catches.
  const std::set<std::string> required = {
      "behaviortree_cpp", "lapkt",  "googletest",         "nlohmann_json",
      "imgui",            "sdl2",   "imgui_node_editor",  "stb",
      "tinyfiledialogs",  "asio",   "jetbrains_mono",     "websocketpp",
  };

  for (const std::string& name : required) {
    const fs::path path = externalDir() / name;
    EXPECT_TRUE(fs::is_directory(path))
        << name << " is not checked in at " << path;
  }
}

TEST(VendoredDependencies, TheFilesTheBuildCompilesAreThere) {
  // One landmark file per dependency, chosen as something the build actually
  // reads. A prune rule that went too far would show up here.
  const std::vector<std::pair<std::string, std::string>> landmarks = {
      {"behaviortree_cpp", "CMakeLists.txt"},
      {"lapkt", "src/model/strips_prob.cxx"},
      {"lapkt", "src/ltl/bit_array.cxx"},
      {"googletest", "googletest/CMakeLists.txt"},
      {"nlohmann_json", "include/nlohmann/json.hpp"},
      {"imgui", "imgui.cpp"},
      {"imgui", "backends/imgui_impl_sdl2.cpp"},
      {"imgui_node_editor", "imgui_node_editor.cpp"},
      {"sdl2", "CMakeLists.txt"},
      {"stb", "stb_image_write.h"},
      {"tinyfiledialogs", "tinyfiledialogs.c"},
      {"jetbrains_mono", "fonts/ttf/JetBrainsMono-Regular.ttf"},
  };

  for (const auto& [dependency, relative] : landmarks) {
    const fs::path path = externalDir() / dependency / relative;
    EXPECT_TRUE(fs::is_regular_file(path)) << "missing " << path;
  }
}

TEST(VendoredDependencies, NoUpstreamVersionControlDirectoriesCameAlong) {
  // A .git directory inside a vendored copy turns the repository into a nested
  // checkout, which does not clone cleanly and carries history nobody wants.
  for (const auto& entry : fs::recursive_directory_iterator(externalDir())) {
    if (entry.is_directory()) {
      EXPECT_NE(entry.path().filename().string(), ".git")
          << "upstream version control left in " << entry.path();
    }
  }
}

TEST(VendoredDependencies, EachCopyRecordsWhereItCameFrom) {
  // Without the repository and tag, nobody can tell what a copy is, whether it
  // has a known vulnerability, or how to update it.
  for (const auto& entry : readManifest()["dependencies"]) {
    const std::string name = entry.at("name").get<std::string>();
    EXPECT_FALSE(entry.at("repository").get<std::string>().empty())
        << name << " records no upstream repository";
    EXPECT_FALSE(entry.at("tag").get<std::string>().empty())
        << name << " records no upstream tag";
    EXPECT_GT(entry.at("files").get<int>(), 0) << name << " records no files";
  }
}
