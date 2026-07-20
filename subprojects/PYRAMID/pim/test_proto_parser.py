import tempfile
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_parser import parse_proto, parse_proto_tree


REPO_ROOT = Path(__file__).resolve().parents[3]
LEGACY_PROTO_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "proto"
MBSE_PROTO_ROOT = REPO_ROOT / "subprojects" / "PYRAMID" / "pim" / "test"


class ProtoParserInteractionTest(unittest.TestCase):
    def test_declaration_comments_are_preserved_as_documentation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "fixture.proto"
            path.write_text(
                '''
syntax = "proto3";
package fixture;

// A command to execute.
message Command {
  // The command identifier.
  string identifier = 1;
}

// Command execution endpoint.
service CommandService {
  // Starts command execution.
  rpc Start(Command) returns (Command);
}
''',
                encoding="utf-8",
            )
            parsed = parse_proto(path)

        self.assertEqual(parsed.messages[0].documentation, ["A command to execute."])
        self.assertEqual(parsed.messages[0].fields[0].documentation,
                         ["The command identifier."])
        self.assertEqual(parsed.services[0].documentation,
                         ["Command execution endpoint."])
        self.assertEqual(parsed.services[0].rpcs[0].documentation,
                         ["Starts command execution."])

    def test_method_option_block_is_parsed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "fixture.proto"
            path.write_text(
                '''
syntax = "proto3";
package fixture;
import "pyramid/options/pyramid.options.proto";

message Request {}
message Ack {}

service Fixture_Service {
  rpc Create(Request) returns (Ack) {
    option (pyramid.options.pyramid_op) = {
      pattern: PUBLISH
      topic: "fixture.request"
      qos: { reliability: RELIABLE durability: VOLATILE depth: 10 }
    };
  }
}
''',
                encoding="utf-8",
            )

            parsed = parse_proto(path)

        rpc = parsed.services[0].rpcs[0]
        self.assertEqual(rpc.pattern, "PUBLISH")
        self.assertEqual(rpc.topic, "fixture.request")
        self.assertEqual(
            rpc.qos,
            {"reliability": "RELIABLE", "durability": "VOLATILE", "depth": 10},
        )

    def test_legacy_tree_has_no_advisory_port_classification(self):
        files = parse_proto_tree(LEGACY_PROTO_ROOT)
        services = [svc for pf in files for svc in pf.services]

        self.assertEqual(len(files), 13)
        self.assertEqual(len(services), 16)
        self.assertTrue(all(svc.port_kind is None for svc in services))
        self.assertFalse(
            any(
                rpc.pattern or rpc.topic or rpc.qos
                for svc in services
                for rpc in svc.rpcs
            )
        )

    def test_mbse_tree_classifies_all_port_services(self):
        files = parse_proto_tree(MBSE_PROTO_ROOT)
        services = [svc for pf in files for svc in pf.services]
        kinds = {svc.port_kind for svc in services}

        self.assertEqual(len(files), 49)
        self.assertEqual(len(services), 40)
        self.assertEqual(kinds, {"request", "information"})
        self.assertEqual(
            sum(1 for svc in services if svc.port_kind == "request"),
            18,
        )
        self.assertEqual(
            sum(1 for svc in services if svc.port_kind == "information"),
            22,
        )


if __name__ == "__main__":
    unittest.main()
