"""Regression tests for MBSE-to-protobuf generation."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mbse.proto_generator import ProtobufGenerator  # noqa: E402


PIM_DIR = Path(__file__).resolve().parent
MODEL = PIM_DIR / "mbse" / "test.json"


class TypeReferenceResolutionTest(unittest.TestCase):
    def test_uuid_reference_wins_over_same_named_interface_block(self):
        """Inheritance must resolve the common Requirement, not a service type."""
        model = json.loads(MODEL.read_text(encoding="utf-8"))

        with tempfile.TemporaryDirectory() as temporary:
            output_dir = Path(temporary)
            ProtobufGenerator(model, str(output_dir)).generate()
            sensors_proto = (output_dir / "pyramid" / "data_model"
                             / "pyramid.data_model.pim_osprey.sensors.proto")
            text = sensors_proto.read_text(encoding="utf-8")

        self.assertIn(
            "pyramid.data_model.common.Requirement base = 1;  "
            "// Inherited from Requirement",
            text)
        self.assertIn(
            "SENRequirement base = 1;  // Inherited from SENRequirement",
            text)


if __name__ == "__main__":
    unittest.main()
