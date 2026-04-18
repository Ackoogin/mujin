#include <StreamingCodec.h>

#include <cstring>
#include <stdexcept>

namespace tactical_objects {

// ---------------------------------------------------------------------------
// Little-endian helpers
// ---------------------------------------------------------------------------
namespace {

void writeU8(std::vector<uint8_t>& buf, uint8_t v) {
  buf.push_back(v);
}

void writeU16LE(std::vector<uint8_t>& buf, uint16_t v) {
  buf.push_back(static_cast<uint8_t>(v & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void writeU32LE(std::vector<uint8_t>& buf, uint32_t v) {
  buf.push_back(static_cast<uint8_t>(v & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

void writeU64LE(std::vector<uint8_t>& buf, uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    buf.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
}

void writeF64LE(std::vector<uint8_t>& buf, double v) {
  uint64_t bits;
  std::memcpy(&bits, &v, sizeof(bits));
  writeU64LE(buf, bits);
}

void writeBytes(std::vector<uint8_t>& buf, const uint8_t* data, size_t len) {
  buf.insert(buf.end(), data, data + len);
}

void writeString16(std::vector<uint8_t>& buf, const std::string& s) {
  uint16_t len = static_cast<uint16_t>(s.size() < 65535u ? s.size() : 65535u);
  writeU16LE(buf, len);
  const uint8_t* p = reinterpret_cast<const uint8_t*>(s.data());
  buf.insert(buf.end(), p, p + len);
}

// ---- read helpers ----

bool readU8(const uint8_t* data, size_t len, size_t& off, uint8_t& out) {
  if (off + 1 > len) return false;
  out = data[off++];
  return true;
}

bool readU16LE(const uint8_t* data, size_t len, size_t& off, uint16_t& out) {
  if (off + 2 > len) return false;
  out = static_cast<uint16_t>(data[off]) |
        (static_cast<uint16_t>(data[off + 1]) << 8);
  off += 2;
  return true;
}

bool readU32LE(const uint8_t* data, size_t len, size_t& off, uint32_t& out) {
  if (off + 4 > len) return false;
  out = static_cast<uint32_t>(data[off]) |
        (static_cast<uint32_t>(data[off+1]) << 8) |
        (static_cast<uint32_t>(data[off+2]) << 16) |
        (static_cast<uint32_t>(data[off+3]) << 24);
  off += 4;
  return true;
}

bool readU64LE(const uint8_t* data, size_t len, size_t& off, uint64_t& out) {
  if (off + 8 > len) return false;
  out = 0;
  for (int i = 0; i < 8; ++i) {
    out |= static_cast<uint64_t>(data[off + i]) << (8 * i);
  }
  off += 8;
  return true;
}

bool readF64LE(const uint8_t* data, size_t len, size_t& off, double& out) {
  uint64_t bits = 0;
  if (!readU64LE(data, len, off, bits)) return false;
  std::memcpy(&out, &bits, sizeof(out));
  return true;
}

bool readString16(const uint8_t* data, size_t len, size_t& off, std::string& out) {
  uint16_t slen = 0;
  if (!readU16LE(data, len, off, slen)) return false;
  if (off + slen > len) return false;
  out.assign(reinterpret_cast<const char*>(data + off), slen);
  off += slen;
  return true;
}

// Encode a single EntityUpdateFrame into bytes (no length prefix)
std::vector<uint8_t> encodeFrameInner(const EntityUpdateFrame& frame) {
  std::vector<uint8_t> buf;
  buf.reserve(64);

  writeU8(buf, frame.message_type);
  // entity UUID: 16 raw bytes
  writeBytes(buf, frame.entity_id.uuid.bytes.data(), 16);
  writeU64LE(buf, frame.version);
  writeF64LE(buf, frame.timestamp);

  if (frame.message_type == STREAM_MSG_ENTITY_DELETE) {
    // delete frame has no field_mask or payload
    return buf;
  }

  writeU16LE(buf, frame.field_mask);

  if ((frame.field_mask & FieldMaskBit::POSITION) && frame.position) {
    writeF64LE(buf, frame.position->lat);
    writeF64LE(buf, frame.position->lon);
    writeF64LE(buf, frame.position->alt);
  }
  if ((frame.field_mask & FieldMaskBit::VELOCITY) && frame.velocity) {
    writeF64LE(buf, frame.velocity->north);
    writeF64LE(buf, frame.velocity->east);
    writeF64LE(buf, frame.velocity->down);
  }
  if ((frame.field_mask & FieldMaskBit::AFFILIATION) && frame.affiliation) {
    writeU8(buf, StreamingCodec::affiliationToOrdinal(*frame.affiliation));
  }
  if ((frame.field_mask & FieldMaskBit::OBJECT_TYPE) && frame.object_type) {
    writeU8(buf, StreamingCodec::objectTypeToOrdinal(*frame.object_type));
  }
  if ((frame.field_mask & FieldMaskBit::CONFIDENCE) && frame.confidence) {
    writeF64LE(buf, *frame.confidence);
  }
  if ((frame.field_mask & FieldMaskBit::LIFECYCLE_STATUS) && frame.lifecycle_status) {
    writeU8(buf, StreamingCodec::lifecycleStatusToOrdinal(*frame.lifecycle_status));
  }
  if ((frame.field_mask & FieldMaskBit::MIL_CLASS) && frame.mil_class) {
    const MilClassProfile& p = *frame.mil_class;
    writeU8(buf, StreamingCodec::battleDimensionToOrdinal(p.battle_dim));
    writeU8(buf, StreamingCodec::affiliationToOrdinal(p.affiliation));
    writeU8(buf, StreamingCodec::milStatusToOrdinal(p.status));
    writeU8(buf, StreamingCodec::echelonToOrdinal(p.echelon));
    writeU8(buf, StreamingCodec::mobilityToOrdinal(p.mobility));
    uint8_t flags = 0;
    if (p.hq)          flags |= 0x01;
    if (p.task_force)  flags |= 0x02;
    if (p.feint_dummy) flags |= 0x04;
    if (p.installation)flags |= 0x08;
    writeU8(buf, flags);
    writeString16(buf, p.role);
    writeString16(buf, p.source_sidc);
  }
  if ((frame.field_mask & FieldMaskBit::BEHAVIOR) && frame.behavior) {
    writeString16(buf, frame.behavior->behavior_pattern);
    writeString16(buf, frame.behavior->operational_state);
  }
  if ((frame.field_mask & FieldMaskBit::IDENTITY_NAME) && frame.identity_name) {
    writeString16(buf, *frame.identity_name);
  }

  return buf;
}

EntityUpdateFrame decodeFrameInner(const uint8_t* data, size_t len) {
  EntityUpdateFrame frame;
  frame.message_type = 0;  // error sentinel
  size_t off = 0;

  uint8_t msg_type = 0;
  if (!readU8(data, len, off, msg_type)) return frame;
  frame.message_type = msg_type;

  // entity UUID: 16 raw bytes
  if (off + 16 > len) { frame.message_type = 0; return frame; }
  std::copy(data + off, data + off + 16, frame.entity_id.uuid.bytes.begin());
  off += 16;

  if (!readU64LE(data, len, off, frame.version)) { frame.message_type = 0; return frame; }
  if (!readF64LE(data, len, off, frame.timestamp)) { frame.message_type = 0; return frame; }

  if (msg_type == STREAM_MSG_ENTITY_DELETE) {
    return frame;
  }

  uint16_t mask = 0;
  if (!readU16LE(data, len, off, mask)) { frame.message_type = 0; return frame; }
  frame.field_mask = mask;

  if (mask & FieldMaskBit::POSITION) {
    Position p;
    if (!readF64LE(data, len, off, p.lat)) { frame.message_type = 0; return frame; }
    if (!readF64LE(data, len, off, p.lon)) { frame.message_type = 0; return frame; }
    if (!readF64LE(data, len, off, p.alt)) { frame.message_type = 0; return frame; }
    frame.position = p;
  }
  if (mask & FieldMaskBit::VELOCITY) {
    Velocity v;
    if (!readF64LE(data, len, off, v.north)) { frame.message_type = 0; return frame; }
    if (!readF64LE(data, len, off, v.east))  { frame.message_type = 0; return frame; }
    if (!readF64LE(data, len, off, v.down))  { frame.message_type = 0; return frame; }
    frame.velocity = v;
  }
  if (mask & FieldMaskBit::AFFILIATION) {
    uint8_t v = 0;
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    frame.affiliation = StreamingCodec::ordinalToAffiliation(v);
  }
  if (mask & FieldMaskBit::OBJECT_TYPE) {
    uint8_t v = 0;
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    frame.object_type = StreamingCodec::ordinalToObjectType(v);
  }
  if (mask & FieldMaskBit::CONFIDENCE) {
    double v = 0.0;
    if (!readF64LE(data, len, off, v)) { frame.message_type = 0; return frame; }
    frame.confidence = v;
  }
  if (mask & FieldMaskBit::LIFECYCLE_STATUS) {
    uint8_t v = 0;
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    frame.lifecycle_status = StreamingCodec::ordinalToLifecycleStatus(v);
  }
  if (mask & FieldMaskBit::MIL_CLASS) {
    MilClassProfile p;
    uint8_t v = 0;
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    p.battle_dim = StreamingCodec::ordinalToBattleDimension(v);
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    p.affiliation = StreamingCodec::ordinalToAffiliation(v);
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    p.status = StreamingCodec::ordinalToMilStatus(v);
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    p.echelon = StreamingCodec::ordinalToEchelon(v);
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    p.mobility = StreamingCodec::ordinalToMobility(v);
    if (!readU8(data, len, off, v)) { frame.message_type = 0; return frame; }
    p.hq          = (v & 0x01) != 0;
    p.task_force  = (v & 0x02) != 0;
    p.feint_dummy = (v & 0x04) != 0;
    p.installation= (v & 0x08) != 0;
    if (!readString16(data, len, off, p.role))       { frame.message_type = 0; return frame; }
    if (!readString16(data, len, off, p.source_sidc)){ frame.message_type = 0; return frame; }
    frame.mil_class = p;
  }
  if (mask & FieldMaskBit::BEHAVIOR) {
    BehaviorComponent bc;
    if (!readString16(data, len, off, bc.behavior_pattern))  { frame.message_type = 0; return frame; }
    if (!readString16(data, len, off, bc.operational_state)) { frame.message_type = 0; return frame; }
    frame.behavior = bc;
  }
  if (mask & FieldMaskBit::IDENTITY_NAME) {
    std::string name;
    if (!readString16(data, len, off, name)) { frame.message_type = 0; return frame; }
    frame.identity_name = name;
  }

  return frame;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// StreamingCodec — public API
// ---------------------------------------------------------------------------

std::vector<uint8_t> StreamingCodec::encodeEntityUpdate(
    const UUIDKey& entity_id,
    uint64_t version,
    double   timestamp,
    uint16_t field_mask,
    const ObjectStore& store)
{
  EntityUpdateFrame frame;
  frame.message_type = STREAM_MSG_ENTITY_UPDATE;
  frame.entity_id  = entity_id;
  frame.version    = version;
  frame.timestamp  = timestamp;
  frame.field_mask = field_mask;

  if (field_mask & FieldMaskBit::POSITION) {
    const auto* kc = store.kinematics().get(entity_id);
    if (kc) frame.position = kc->position;
  }
  if (field_mask & FieldMaskBit::VELOCITY) {
    const auto* kc = store.kinematics().get(entity_id);
    if (kc) frame.velocity = kc->velocity;
  }
  if (field_mask & FieldMaskBit::AFFILIATION) {
    const auto* mc = store.milclass().get(entity_id);
    if (mc) frame.affiliation = mc->profile.affiliation;
  }
  if (field_mask & FieldMaskBit::OBJECT_TYPE) {
    const auto* rec = store.getRecord(entity_id);
    if (rec) frame.object_type = rec->type;
  }
  if (field_mask & FieldMaskBit::CONFIDENCE) {
    const auto* qc = store.quality().get(entity_id);
    if (qc) frame.confidence = qc->confidence;
  }
  if (field_mask & FieldMaskBit::LIFECYCLE_STATUS) {
    const auto* lc = store.lifecycle().get(entity_id);
    if (lc) frame.lifecycle_status = lc->status;
  }
  if (field_mask & FieldMaskBit::MIL_CLASS) {
    const auto* mc = store.milclass().get(entity_id);
    if (mc) frame.mil_class = mc->profile;
  }
  if (field_mask & FieldMaskBit::BEHAVIOR) {
    const auto* bc = store.behaviors().get(entity_id);
    if (bc) frame.behavior = *bc;
  }
  if (field_mask & FieldMaskBit::IDENTITY_NAME) {
    const auto* ic = store.identities().get(entity_id);
    if (ic) frame.identity_name = ic->name;
  }

  return encodeFrameInner(frame);
}

std::vector<uint8_t> StreamingCodec::encodeEntityUpdateFrame(const EntityUpdateFrame& frame) {
  return encodeFrameInner(frame);
}

EntityUpdateFrame StreamingCodec::decodeEntityUpdate(const uint8_t* data, size_t len) {
  return decodeFrameInner(data, len);
}

std::vector<uint8_t> StreamingCodec::encodeBatchFrame(
    const std::vector<EntityUpdateFrame>& frames,
    double tick_timestamp)
{
  // Encode each frame independently first
  std::vector<std::vector<uint8_t>> encoded;
  encoded.reserve(frames.size());
  for (auto& f : frames) {
    encoded.push_back(encodeFrameInner(f));
  }

  // batch header: type(1)=0x03 + entity_count(4) + tick_timestamp(8) = 13 bytes
  std::vector<uint8_t> buf;
  uint32_t count = static_cast<uint32_t>(frames.size());
  buf.reserve(13 + frames.size() * 64);
  writeU8(buf, 0x03);
  writeU32LE(buf, count);
  writeF64LE(buf, tick_timestamp);

  for (auto& e : encoded) {
    writeU32LE(buf, static_cast<uint32_t>(e.size()));
    writeBytes(buf, e.data(), e.size());
  }

  return buf;
}

std::vector<EntityUpdateFrame> StreamingCodec::decodeBatchFrame(
    const uint8_t* data, size_t len)
{
  std::vector<EntityUpdateFrame> result;
  size_t off = 0;

  uint8_t frame_type = 0;
  if (!readU8(data, len, off, frame_type)) return result;
  if (frame_type != 0x03) return result;

  uint32_t count = 0;
  if (!readU32LE(data, len, off, count)) return result;

  double tick_ts = 0.0;
  if (!readF64LE(data, len, off, tick_ts)) return result;

  for (uint32_t i = 0; i < count; ++i) {
    uint32_t frame_size = 0;
    if (!readU32LE(data, len, off, frame_size)) break;
    if (off + frame_size > len) break;
    EntityUpdateFrame f = decodeFrameInner(data + off, frame_size);
    off += frame_size;
    if (f.message_type != 0) {
      result.push_back(f);
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Enum ordinal helpers
// ---------------------------------------------------------------------------

uint8_t StreamingCodec::affiliationToOrdinal(Affiliation a) {
  return static_cast<uint8_t>(a);
}
Affiliation StreamingCodec::ordinalToAffiliation(uint8_t v) {
  if (v > static_cast<uint8_t>(Affiliation::AssumedFriend)) return Affiliation::Unknown;
  return static_cast<Affiliation>(v);
}

uint8_t StreamingCodec::objectTypeToOrdinal(ObjectType t) {
  return static_cast<uint8_t>(t);
}
ObjectType StreamingCodec::ordinalToObjectType(uint8_t v) {
  if (v > static_cast<uint8_t>(ObjectType::Zone)) return ObjectType::Platform;
  return static_cast<ObjectType>(v);
}

uint8_t StreamingCodec::lifecycleStatusToOrdinal(LifecycleStatus s) {
  return static_cast<uint8_t>(s);
}
LifecycleStatus StreamingCodec::ordinalToLifecycleStatus(uint8_t v) {
  if (v > static_cast<uint8_t>(LifecycleStatus::Retired)) return LifecycleStatus::Active;
  return static_cast<LifecycleStatus>(v);
}

uint8_t StreamingCodec::battleDimensionToOrdinal(BattleDimension d) {
  return static_cast<uint8_t>(d);
}
BattleDimension StreamingCodec::ordinalToBattleDimension(uint8_t v) {
  if (v > static_cast<uint8_t>(BattleDimension::SOF)) return BattleDimension::Ground;
  return static_cast<BattleDimension>(v);
}

uint8_t StreamingCodec::milStatusToOrdinal(MilStatus s) {
  return static_cast<uint8_t>(s);
}
MilStatus StreamingCodec::ordinalToMilStatus(uint8_t v) {
  if (v > static_cast<uint8_t>(MilStatus::Known)) return MilStatus::Present;
  return static_cast<MilStatus>(v);
}

uint8_t StreamingCodec::echelonToOrdinal(Echelon e) {
  return static_cast<uint8_t>(e);
}
Echelon StreamingCodec::ordinalToEchelon(uint8_t v) {
  if (v > static_cast<uint8_t>(Echelon::ArmyGroup)) return Echelon::Team;
  return static_cast<Echelon>(v);
}

uint8_t StreamingCodec::mobilityToOrdinal(Mobility m) {
  return static_cast<uint8_t>(m);
}
Mobility StreamingCodec::ordinalToMobility(uint8_t v) {
  if (v > static_cast<uint8_t>(Mobility::Amphibious)) return Mobility::None;
  return static_cast<Mobility>(v);
}

}  // namespace tactical_objects
