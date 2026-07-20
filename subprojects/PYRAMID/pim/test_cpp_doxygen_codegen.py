import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cpp.types_gen import CppTypesGenerator
from proto_parser import parse_proto


class CppDoxygenCodegenTest(unittest.TestCase):
    def test_proto_documentation_is_emitted_as_doxygen_comments(self):
        source = '''
syntax = "proto3";
package example.docs;

// The available operating modes.
enum Mode {
  // The component is idle.
  MODE_IDLE = 0;
}

// A command sent to the component.
message Command {
  // Identifies the command.
  string identifier = 1;

  // Mutually exclusive payloads.
  oneof payload {
    // The requested mode.
    Mode mode = 2;
  }
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            proto = root / 'docs.proto'
            proto.write_text(source, encoding='utf-8')
            CppTypesGenerator([parse_proto(proto)]).generate(root)
            generated = (root / 'example_docs_types.hpp').read_text(encoding='utf-8')

        self.assertIn('/// \\brief The available operating modes.\n'
                      'enum class Mode', generated)
        self.assertIn('    /// \\brief The component is idle.\n'
                      '    Idle = 0,', generated)
        self.assertIn('/// \\brief A command sent to the component.\n'
                      'struct Command', generated)
        self.assertIn('    /// \\brief Identifies the command.\n'
                      '    std::string identifier', generated)
        self.assertIn('    /// \\brief Mutually exclusive payloads.\n'
                      '    // oneof payload', generated)
        self.assertIn('    /// \\brief The requested mode.\n'
                      '    tl::optional<Mode> mode;', generated)


if __name__ == '__main__':
    unittest.main()
