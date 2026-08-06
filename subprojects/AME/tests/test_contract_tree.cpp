// Guards the contract tree that AME owns under subprojects/AME/contracts/proto.
//
// AME needs a handful of PYRAMID contracts in order to generate its bindings.
// Those same files also exist inside PYRAMID's proof layer, which is where AME
// used to read them from. AME now keeps its own copies so that it can be built
// with the proof layer switched off, and that leaves two copies of each shared
// file in the repository.
//
// These tests exist so the two copies cannot drift apart unnoticed. If someone
// changes a data model inside PYRAMID and AME's copy is not updated to match,
// the build would keep working and the two configurations would quietly
// generate different bindings. The comparison below turns that into a test
// failure instead.
//
// If a test here fails, the fix is normally to copy the changed file from
// PYRAMID's proof contracts into subprojects/AME/contracts/proto, and to check
// that AME still builds against it.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

/// Files AME keeps its own copy of, relative to the root of a contract tree.
/// Every entry must exist in AME's tree; entries that also exist in PYRAMID's
/// proof tree must match it byte for byte.
const std::vector<std::string>& sharedContractFiles() {
  static const std::vector<std::string> files = {
      "pyramid/options/pyramid.options.proto",
      "pyramid/data_model/pyramid.data_model.base.proto",
      "pyramid/data_model/pyramid.data_model.common.proto",
      "pyramid/data_model/pyramid.data_model.autonomy.proto",
      "pyramid/components/pyramid.components.autonomy_backend.services.provided.proto",
      "pyramid/components/pyramid.components.agra_ma_grounding.services.provided.proto",
  };
  return files;
}

std::string readFile(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

fs::path ameContractRoot() { return fs::path(AME_CONTRACT_PROTO_DIR); }

fs::path pyramidProofContractRoot() { return fs::path(PYRAMID_PROOF_PROTO_DIR); }

}  // namespace

TEST(AmeContractTree, ContainsEveryFileAmeNeedsToGenerateBindings) {
  for (const std::string& relative : sharedContractFiles()) {
    const fs::path path = ameContractRoot() / relative;
    EXPECT_TRUE(fs::exists(path))
        << "AME's contract tree is missing " << relative
        << ". Binding generation will fail when PYRAMID's proof contracts are "
           "not built. Expected it at " << path.string();
  }
}

TEST(AmeContractTree, SharedFilesAreIdenticalToPyramidProofContracts) {
  const fs::path proof_root = pyramidProofContractRoot();
  if (!fs::exists(proof_root)) {
    GTEST_SKIP() << "PYRAMID proof contracts are not present in this checkout, "
                    "so there is nothing to compare against.";
  }

  for (const std::string& relative : sharedContractFiles()) {
    const fs::path ame_path = ameContractRoot() / relative;
    const fs::path proof_path = proof_root / relative;

    if (!fs::exists(proof_path)) {
      // A contract that only AME declares is legitimate; it simply has no
      // counterpart to drift from.
      continue;
    }
    ASSERT_TRUE(fs::exists(ame_path))
        << "AME's contract tree is missing " << relative;

    EXPECT_EQ(readFile(ame_path), readFile(proof_path))
        << "The AME and PYRAMID copies of " << relative << " have diverged.\n"
        << "AME copy:     " << ame_path.string() << '\n'
        << "PYRAMID copy: " << proof_path.string() << '\n'
        << "Copy the PYRAMID version over the AME version, then confirm AME "
           "still builds against it.";
  }
}

TEST(AmeContractTree, EveryImportResolvesInsideAmesOwnTree) {
  // A contract tree is only self-contained if each file's imports can be
  // satisfied from the same tree. Anything else would reintroduce the
  // dependency on PYRAMID's proof layer that this tree exists to remove.
  for (const std::string& relative : sharedContractFiles()) {
    const fs::path path = ameContractRoot() / relative;
    if (!fs::exists(path)) {
      continue;  // Reported by the first test.
    }

    std::ifstream stream(path);
    std::string line;
    while (std::getline(stream, line)) {
      const std::string prefix = "import \"";
      const size_t start = line.find(prefix);
      if (line.rfind("import", 0) != 0 || start == std::string::npos) {
        continue;
      }
      const size_t open_quote = start + prefix.size();
      const size_t close_quote = line.find('"', open_quote);
      ASSERT_NE(close_quote, std::string::npos)
          << "Malformed import line in " << relative << ": " << line;

      const std::string imported =
          line.substr(open_quote, close_quote - open_quote);

      // Imports of protobuf's own well-known types are resolved by protoc from
      // its bundled include path, not from this tree.
      if (imported.rfind("google/protobuf/", 0) == 0) {
        continue;
      }

      EXPECT_TRUE(fs::exists(ameContractRoot() / imported))
          << relative << " imports " << imported
          << ", which is not present in AME's contract tree. Add it to "
             "subprojects/AME/contracts/proto and to sharedContractFiles() in "
             "this test.";
    }
  }
}
