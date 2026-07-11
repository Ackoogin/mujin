#!/usr/bin/env python3
"""UCI 2.5 OMS-JSON codec backend.

OMS JSON is deliberately not a generic proto-to-JSON projection.  It accepts
only the UCI-shaped ``pyramid.data_model.uci`` contract and emits the UCI
global-element envelope used by Sleet.
"""

from pathlib import Path
from typing import List
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import codec_backends
from proto_parser import ProtoTypeIndex
from cpp.oms_json_codec_gen import CppOmsJsonCodecGenerator
from ada.oms_json_codec_gen import AdaOmsJsonCodecGenerator


class OmsJsonBackend(codec_backends.CodecBackend):
    @property
    def name(self) -> str:
        return 'oms_json'

    @property
    def content_type(self) -> str:
        return 'application/oms-json'

    def generate_cpp(self, index: ProtoTypeIndex, output_dir: Path, **kwargs) -> List[Path]:
        return CppOmsJsonCodecGenerator(
            index, naming_policy=kwargs.get('naming_policy')
        ).generate(output_dir)

    def generate_ada(self, index: ProtoTypeIndex, output_dir: Path) -> List[Path]:
        return AdaOmsJsonCodecGenerator(index).generate(output_dir)


codec_backends.register(OmsJsonBackend())
