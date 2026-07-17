#!/usr/bin/env python3
"""Generate a PYRAMID interaction seam from a converted XSD profile.

A "seam" is a self-contained proto contract directory that pairs a
generated data model (from ``xsd2proto.py``) with component service
definitions carrying PYRAMID port-grammar annotations (``pyramid_op``:
PUBLISH/SUBSCRIBE topics with QoS), so the binding generator can produce
codecs, marshal layers, and interaction facades for it. The hand-authored
examples of the shape this script reproduces are ``pim/uci_p1_seam/`` and
``pim/agra_p2_seam/``; this script exists because profile P3 (the full
A-GRA Core MMS, 343 root messages across four interfaces) is far too large
to hand-author.

Inputs:

* the profile manifest (``pim/uci_profiles/<profile>.json``) -- names the
  drop and proto package;
* the interface/direction table
  (``pim/uci_profiles/<profile>_interfaces.json``) -- which messages
  travel on which compliance-document interface (C2, MS, P2P, VI) and in
  which direction. Direction is relative to the interface's far side (the
  C2 station, mission system, peer, or vehicle): 'in' means the Mission
  Autonomy system sends the message, 'out' means it receives it, 'inout'
  both;
* the converted tree (``pim/uci_generated_p3/<drop>/``) -- the data-model
  proto and its ``wire_names.json`` sidecar (whose ``roots`` map provides
  element -> message-type names).
* the P2 seam (``pim/agra_p2_seam/``) -- compatibility service and
  port-grammar definitions that P3 copies byte for byte.

Output layout (all deterministic; re-running produces identical bytes):

* byte-for-byte copies of the data-model proto, ``wire_names.json``, and
  the shared options contract (copies, not symlinks, for native Windows
  checkouts -- same reasoning as ``pim/agra_p2_seam/README.md``);
* a small port-grammar support proto (Identifier/Ack/Query) in its own
  package so it cannot clobber the data-model types header;
* ``binding_metadata.json`` carrying the drop identity;
* per-interface component service protos, provided and consumed.
* byte-for-byte P2 compatibility service and port-grammar protos.

Service derivation rules (mirroring the P1/P2 seam grammar):

* A message X ending in "Command" or "Request" whose partner "XStatus"
  is on the same interface forms a correlated command pair (the A-GRA
  Command-2 pattern). The pair becomes one Request/Entity service
  with Create/Read/Update/Cancel rpcs. When the far side commands the MA
  system (command direction 'out'), the service is provided: commands are
  SUBSCRIBEd, statuses PUBLISHed. When the MA system is the commander
  (direction 'in'), the mirrored service is consumed: commands PUBLISHed,
  statuses SUBSCRIBEd. 'inout' emits both.
* Every other message becomes a single-variant Information service:
  provided (PUBLISH) when the MA system sends it, consumed (SUBSCRIBE)
  when it receives it, both for 'inout'.
* Topics are the bare XSD global element names (the LA-CAL/Sleet routing
  key, as in the P2 seam). All operations are RELIABLE/VOLATILE with
  queue depth 10 (the P2 seam's approved floor).
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

PIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PIM_DIR))

# The same field-name derivation the converter applies, including its
# C++ reserved-word escape: the element 'Operator' becomes the field
# 'operator_', not the bare keyword. Confirmed live before sharing this:
# a seam-local snake_case without the escape emitted 'w.operator' in the
# generated OMS codec plugin's service-wire structs, which MSVC rejects.
from xsd2proto import snake_case

QOS = 'qos: { reliability: RELIABLE durability: VOLATILE depth: 10 }'

# The four interface columns of the compliance document's Table 3-1, in
# emission order, with the component-package segment each becomes.
INTERFACE_PACKAGES = [
    ('C2', 'c2'),
    ('MS', 'ms'),
    ('P2P', 'p2p'),
    ('VI', 'vi'),
]


def pair_up(elements: dict) -> tuple:
    """Split one interface's element->direction map into command pairs and
    standalone information elements.

    Returns (pairs, information) where pairs is a list of
    (command_element, status_element, command_direction) and information
    is a list of (element, direction).
    """
    names = set(elements)
    paired_status = set()
    pairs = []
    for elem in sorted(names):
        if not (elem.endswith('Command') or elem.endswith('Request')):
            continue
        status = elem + 'Status'
        if status in names:
            pairs.append((elem, status, elements[elem]))
            paired_status.add(status)
    consumed_by_pairs = {p[0] for p in pairs} | paired_status
    information = [(e, d) for e, d in sorted(elements.items())
                   if e not in consumed_by_pairs]
    return pairs, information


def _command_service(out, dm_pkg, pg_pkg, roots, cmd, status, provided):
    """One Request/Entity command service. ``provided`` selects the
    executor side (SUBSCRIBE commands, PUBLISH statuses); the consumer
    side inverts every pattern, mirroring the hand-authored
    provided/consumed pairs in pim/uci_p1_seam and pim/agra_example."""
    svc = f'{cmd}_Service'
    cmd_type = f'{dm_pkg}.{roots[cmd]}'
    status_type = f'{dm_pkg}.{roots[status]}'
    sub = 'SUBSCRIBE' if provided else 'PUBLISH'
    pub = 'PUBLISH' if provided else 'SUBSCRIBE'
    out.append(f'message {svc}_Request {{')
    out.append('  oneof payload {')
    out.append(f'    {cmd_type} {snake_case(cmd)} = 1;')
    out.append(f'    {status_type} update = 2;')
    out.append('  }')
    out.append('}')
    out.append('')
    out.append(f'message {svc}_Entity {{')
    out.append('  oneof payload {')
    out.append(f'    {status_type} {snake_case(status)} = 1;')
    out.append('  }')
    out.append('}')
    out.append('')
    out.append(f'service {svc} {{')
    out.append(f'  rpc Create({svc}_Request) returns ({pg_pkg}.Ack) {{')
    out.append(f'    option (pyramid.options.pyramid_op) = {{ pattern: {sub} topic: "{cmd}" {QOS} }};')
    out.append('  }')
    out.append(f'  rpc Read({pg_pkg}.Query) returns (stream {svc}_Entity) {{')
    out.append(f'    option (pyramid.options.pyramid_op) = {{ pattern: {pub} topic: "{status}" {QOS} }};')
    out.append('  }')
    out.append(f'  rpc Update({svc}_Entity) returns ({pg_pkg}.Ack) {{')
    out.append(f'    option (pyramid.options.pyramid_op) = {{ pattern: {sub} topic: "{cmd}" {QOS} }};')
    out.append('  }')
    out.append(f'  rpc Cancel({pg_pkg}.Identifier) returns ({pg_pkg}.Ack) {{')
    out.append(f'    option (pyramid.options.pyramid_op) = {{ pattern: {sub} topic: "{cmd}" {QOS} }};')
    out.append('  }')
    out.append('}')
    out.append('')


def _information_service(out, dm_pkg, roots, elem, provided):
    """One single-variant Information service (PUBLISH when provided,
    SUBSCRIBE when consumed)."""
    svc = f'{elem}_Service'
    pattern = 'PUBLISH' if provided else 'SUBSCRIBE'
    out.append(f'message {svc}_Information {{')
    out.append('  oneof payload {')
    out.append(f'    {dm_pkg}.{roots[elem]} {snake_case(elem)} = 1;')
    out.append('  }')
    out.append('}')
    out.append('')
    out.append(f'service {svc} {{')
    out.append(f'  rpc Read(google.protobuf.Empty) returns (stream {svc}_Information) {{')
    out.append(f'    option (pyramid.options.pyramid_op) = {{ pattern: {pattern} topic: "{elem}" {QOS} }};')
    out.append('  }')
    out.append('}')
    out.append('')


def render_component(seam_pkg, iface, iface_pkg, role, dm_pkg, dm_file,
                     pg_pkg, pg_file, roots, pairs, information):
    """Render one component proto (one interface, one role), or None when
    the role has no services on this interface."""
    provided = role == 'provided'
    body = []
    for cmd, status, direction in pairs:
        # Command direction 'out' means the far side commands the MA
        # system, which therefore provides the executor service; 'in'
        # means the MA system is the commander and consumes the far
        # side's service. 'inout' appears on both sides.
        if provided and direction in ('out', 'inout'):
            _command_service(body, dm_pkg, pg_pkg, roots, cmd, status, True)
        elif not provided and direction in ('in', 'inout'):
            _command_service(body, dm_pkg, pg_pkg, roots, cmd, status, False)
    has_information = False
    for elem, direction in information:
        if provided and direction in ('in', 'inout'):
            _information_service(body, dm_pkg, roots, elem, True)
            has_information = True
        elif not provided and direction in ('out', 'inout'):
            _information_service(body, dm_pkg, roots, elem, False)
            has_information = True
    if not body:
        return None
    has_commands = f'{pg_pkg}.Ack' in '\n'.join(body)
    out = ['syntax = "proto3";', '']
    out.append(f'package {seam_pkg}.{iface_pkg}.services.{role};')
    out.append('')
    out.append(f'// {iface} interface of Table 3-1 (MA L1 Compliance '
               f'Document), {role} side.')
    out.append('// Generated by pim/gen_interaction_seam.py -- do not edit '
               'by hand.')
    out.append('')
    if has_information:
        out.append('import "google/protobuf/empty.proto";')
    out.append('import "pyramid/options/pyramid.options.proto";')
    out.append(f'import "pyramid/data_model/{dm_file}";')
    if has_commands:
        out.append(f'import "pyramid/data_model/{pg_file}";')
    out.append('')
    out.extend(body)
    return '\n'.join(out).rstrip('\n') + '\n'


README_TEMPLATE = '''# A-GRA P3 (Core MMS) interaction seam

Generated by `pim/gen_interaction_seam.py` -- **do not edit by hand**;
`pim/test_agra_p3_seam.py` regenerates this directory and fails on any
byte difference. Inputs, derivation rules, and the evidence boundary are
documented in the generator's docstring and in
[`pim/uci_profiles/README.md`](../uci_profiles/README.md) ("P3
conversion-scope decision").

This directory pairs the converted A-GRA 5.0a Core MMS data model
({message_count} messages from {root_count} roots; byte-for-byte copy of
`pim/uci_generated_p3/`) with per-interface component service protos
(C2, MS, P2P, VI -- provided and consumed sides, {service_count} services
derived from Table 3-1) carrying the PYRAMID pubsub/rpc port-grammar annotations. It also
contains P2's service API byte for byte, so unchanged P2 client source can
use P3 as its larger contract. Topics
are the bare XSD global element names; correlated Command/Status pairs
follow the Request/Entity pattern; everything else is a
single-variant Information service. All operations are RELIABLE/VOLATILE
with queue depth 10.

No MA L1 compliance claim follows from this seam's existence; unlike the
P1/P2 seams it has no offline fidelity or live interop evidence.
'''

PORT_GRAMMAR_TEMPLATE = '''syntax = "proto3";

package {pg_pkg};

// Small local port-grammar support types -- not XSD content, so they are
// not part of the xsd2proto-converted tree this seam copies in. They live
// in a distinct package (rather than reopening the data-model package)
// because the C++ type-header emitter maps one generated header per
// package name: a second file declaring the data-model package would
// silently clobber that tree's generated header instead of merging with
// it. Mirrors pim/uci_p1_seam's identical definitions.
message Identifier {{ string id = 1; }}
message Ack {{ bool success = 1; }}
message Query {{}}
'''


def generate(manifest_path: Path, interfaces_path: Path, tree_dir: Path,
             options_path: Path, p2_compat_dir: Path, out_dir: Path) -> list:
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    table = json.loads(interfaces_path.read_text(encoding='utf-8'))
    if table.get('profile') != manifest.get('profile'):
        sys.exit(f'gen_interaction_seam: interface table profile '
                 f'{table.get("profile")!r} does not match manifest '
                 f'{manifest.get("profile")!r}')

    dm_pkg = manifest['proto_package']
    dm_file = f'{dm_pkg}.proto'
    # pyramid.data_model.uci -> pyramid.data_model.uci_port_grammar (the
    # P1 seam's convention, applied to this profile's package).
    pg_pkg = dm_pkg + '_port_grammar'
    pg_file = f'{dm_pkg}.port_grammar.proto'
    # Component packages hang off the profile's own segment so they cannot
    # collide with the P2 seam's pyramid.components.agra.* namespace.
    seam_pkg = 'pyramid.components.' + dm_pkg.rsplit('.', 1)[1]

    src_proto = tree_dir / 'pyramid' / 'data_model' / dm_file
    src_wire = tree_dir / 'wire_names.json'
    for p in (src_proto, src_wire, options_path):
        if not p.is_file():
            sys.exit(f'gen_interaction_seam: missing input {p}')
    roots = json.loads(src_wire.read_text(encoding='utf-8'))['roots']

    written = []

    def emit(rel: str, content: str) -> None:
        path = out_dir / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8', newline='\n')
        written.append(rel)

    def copy(rel: str, src: Path) -> None:
        path = out_dir / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, path)
        written.append(rel)

    copy(f'pyramid/data_model/{dm_file}', src_proto)
    copy('wire_names.json', src_wire)
    copy('pyramid/options/pyramid.options.proto', options_path)
    emit('binding_metadata.json', json.dumps({
        'drop': manifest['drop'],
        'owp_init_schema': manifest['schema_version'].split('-')[0],
        'schema_version': manifest['schema_version'],
    }, indent=2, sort_keys=True) + '\n')

    # P3 is a source-level extension of P2, not a parallel client contract.
    # Its data model retains P2's package and contains every P2 type. Copy the
    # P2 port grammar and services so P2 imports and facade names remain valid.
    p2_port_grammar = p2_compat_dir / 'pyramid' / 'data_model' / pg_file
    if not p2_port_grammar.is_file():
        sys.exit(f'gen_interaction_seam: missing P2 compatibility port '
                 f'grammar {p2_port_grammar}')
    copy(f'pyramid/data_model/{pg_file}', p2_port_grammar)
    p2_component_dir = p2_compat_dir / 'pyramid' / 'components'
    p2_services = sorted(p2_component_dir.glob('*.proto'))
    if not p2_services:
        sys.exit(f'gen_interaction_seam: no P2 compatibility services under '
                 f'{p2_component_dir}')
    for source in p2_services:
        copy(f'pyramid/components/{source.name}', source)

    service_count = 0
    for iface, iface_pkg in INTERFACE_PACKAGES:
        elements = table['interfaces'].get(iface, {})
        missing = sorted(e for e in elements if e not in roots)
        if missing:
            sys.exit(f'gen_interaction_seam: {iface} names elements absent '
                     f'from the converted tree: {missing}')
        pairs, information = pair_up(elements)
        for role in ('provided', 'consumed'):
            content = render_component(
                seam_pkg, iface, iface_pkg, role, dm_pkg, dm_file,
                pg_pkg, pg_file, roots, pairs, information)
            if content is not None:
                emit(f'pyramid/components/{seam_pkg}.{iface_pkg}.services.'
                     f'{role}.proto', content)
                service_count += content.count('\nservice ')

    closure = json.loads(
        (tree_dir / 'closure_report.json').read_text(encoding='utf-8'))
    emit('README.md', README_TEMPLATE.format(
        message_count=closure['counts']['messages'],
        root_count=len(roots),
        service_count=service_count))

    return written


def main() -> None:
    ap = argparse.ArgumentParser(
        description='Generate a PYRAMID interaction seam (data model + '
                    'per-interface service protos) from a converted XSD '
                    'profile and its Table 3-1 interface table.')
    ap.add_argument('--manifest',
                    default=PIM_DIR / 'uci_profiles' / 'p3_agra_core_mms.json',
                    type=Path)
    ap.add_argument('--interfaces',
                    default=PIM_DIR / 'uci_profiles'
                    / 'p3_agra_core_mms_interfaces.json',
                    type=Path)
    ap.add_argument('--tree',
                    default=PIM_DIR / 'uci_generated_p3' / 'agra_5_0a',
                    type=Path,
                    help='xsd2proto output tree (holds the data-model proto '
                         'and wire_names.json)')
    ap.add_argument('--options',
                    default=PIM_DIR.parent / 'proto' / 'pyramid' / 'options'
                    / 'pyramid.options.proto',
                    type=Path)
    ap.add_argument('--p2-compat-seam',
                    default=PIM_DIR / 'agra_p2_seam', type=Path,
                    help='P2 seam whose data-model package and service API '
                    'P3 preserves')
    ap.add_argument('--out', default=PIM_DIR / 'agra_p3_seam', type=Path)
    args = ap.parse_args()

    written = generate(args.manifest, args.interfaces, args.tree,
                       args.options, args.p2_compat_seam, args.out)
    for rel in written:
        print(f'wrote {args.out / rel}')


if __name__ == '__main__':
    main()
