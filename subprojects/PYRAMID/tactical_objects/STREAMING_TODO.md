# Tactical Objects Entity Streaming Optimisation Plan

This plan addresses the need for other components to efficiently stream constant updates of entities matching an interest. It is organised into five steps, each building on the previous.

## Step 1 — Wire InterestManager to Event Publication

**Goal:** When an entity is created, updated, or deleted, evaluate it against active interests and only publish to matching subscribers. Today `InterestManager` stores criteria passively; this step makes it drive filtered push.

### Tasks

- [ ] Add a dirty-entity tracking set to `TacticalObjectsRuntime`
  - On every mutation (`createObject`, `updateObject`, `deleteObject`, `processObservation`, `processObservationBatch`), insert the affected `UUIDKey` into a `std::unordered_set<UUIDKey> dirty_entities_`
  - Clear the set after each publish cycle

- [ ] Add `InterestManager::matchesInterest(const InterestCriteria&, const EntityRecord&, const ObjectStore&) const -> bool`
  - Evaluate affiliation filter against entity's `MilClassComponent` or direct affiliation
  - Evaluate object_type filter against `EntityRecord::type`
  - Evaluate area (BoundingBox) filter against entity's `KinematicsComponent::position`
  - Evaluate behavior_pattern filter against `BehaviorComponent`
  - Evaluate minimum_confidence filter against `QualityComponent`
  - Evaluate time_window filters against `LifecycleComponent::last_update_timestamp`

- [ ] Add `InterestManager::matchingInterests(const EntityRecord&, const ObjectStore&) const -> std::vector<UUIDKey>`
  - Iterate active interests, return IDs of those whose criteria match the entity

- [ ] Add subscriber registration to `TacticalObjectsRuntime`
  - Define callback type: `using EntityUpdateCallback = std::function<void(const UUIDKey& interest_id, const UUIDKey& entity_id)>`
  - `registerStreamSubscriber(const UUIDKey& interest_id, EntityUpdateCallback cb) -> SubscriptionHandle`
  - `removeStreamSubscriber(SubscriptionHandle handle)`
  - Store subscribers in a map keyed by interest ID

- [ ] Implement filtered publish in `on_tick()`
  - For each entity in `dirty_entities_`, call `matchingInterests()`
  - For each matching interest, invoke registered callbacks
  - Clear dirty set

- [ ] Add `subscribe_interest` PCL service to `TacticalObjectsComponent`
  - Accepts interest criteria, registers with `InterestManager`, returns interest ID
  - Optionally opens a dedicated publisher channel per interest (or uses a shared `entity_updates` publisher with interest ID tagging)

- [ ] Unit tests
  - `Test_InterestManager.cpp`: add tests for `matchesInterest` with each filter dimension
  - `Test_TacticalObjectsRuntime.cpp`: add tests for dirty tracking, callback invocation, and interest-filtered publication
  - Verify that unmatched entities are not published
  - Verify that expired/cancelled interests stop receiving updates

### Files touched

- `include/interest/InterestManager.h` / `src/interest/InterestManager.cpp`
- `include/TacticalObjectsRuntime.h` / `src/TacticalObjectsRuntime.cpp`
- `include/TacticalObjectsComponent.h` / `src/TacticalObjectsComponent.cpp`
- `tests/tactical_objects/Test_InterestManager.cpp`
- `tests/tactical_objects/Test_TacticalObjectsRuntime.cpp`

---

## Step 2 — Add a Binary Streaming Codec

**Goal:** Introduce a compact binary codec for the high-rate entity update path. Keep JSON for control-plane services (CRUD, query). No new dependencies — just packed structs and `memcpy`.

### Wire Format

```
Offset  Size  Field
0       1     message_type   (0x01 = entity_update, 0x02 = entity_delete)
1       16    entity_uuid    (raw bytes, no string conversion)
17      8     version        (uint64_t, little-endian)
25      8     timestamp      (double, IEEE 754)
33      2     field_mask     (uint16_t, bitfield identifying present fields)
35      var   payload        (only fields indicated by field_mask)
```

Field mask bits:

| Bit | Field | Payload bytes |
|-----|-------|---------------|
| 0   | position (lat, lon, alt) | 24 (3 × double) |
| 1   | velocity (north, east, down) | 24 (3 × double) |
| 2   | affiliation | 1 (uint8_t enum ordinal) |
| 3   | object_type | 1 (uint8_t enum ordinal) |
| 4   | confidence | 8 (double) |
| 5   | lifecycle_status | 1 (uint8_t enum ordinal) |
| 6   | mil_class_profile | 12 (packed: battle_dim(1) + affiliation(1) + status(1) + echelon(1) + mobility(1) + flags(1) + role_len(2) + role(var) + sidc_len(2) + sidc(var)) |
| 7   | behavior | var (pattern_len(2) + pattern + state_len(2) + state) |
| 8   | identity_name | var (name_len(2) + name) |

### Tasks

- [ ] Create `include/StreamingCodec.h` and `src/StreamingCodec.cpp`
  - `StreamingCodec::encodeEntityUpdate(const UUIDKey&, uint64_t version, double timestamp, uint16_t field_mask, const ObjectStore&) -> std::vector<uint8_t>`
  - `StreamingCodec::decodeEntityUpdate(const uint8_t* data, size_t len) -> EntityUpdateFrame`
  - Define `EntityUpdateFrame` struct with optional fields matching the mask
  - All multi-byte integers little-endian
  - Use `static_assert` on struct sizes where applicable

- [ ] Add enum ordinal helpers
  - `static uint8_t affiliationToOrdinal(Affiliation)` / `static Affiliation ordinalToAffiliation(uint8_t)`
  - Same for `ObjectType`, `LifecycleStatus`, `BattleDimension`, `MilStatus`, `Echelon`, `Mobility`
  - These are O(1) unlike the string if-chains

- [ ] Add `StreamingCodec::encodeBatchFrame(const std::vector<EntityUpdateFrame>&) -> std::vector<uint8_t>`
  - Frame header: frame_type(1) + entity_count(4) + tick_timestamp(8)
  - Followed by concatenated per-entity payloads
  - Allows a single PCL message per tick

- [ ] Add `StreamingCodec::decodeBatchFrame(const uint8_t* data, size_t len) -> std::vector<EntityUpdateFrame>`

- [ ] Unit tests: `tests/tactical_objects/Test_StreamingCodec.cpp`
  - Round-trip encode/decode with every field combination
  - Verify wire sizes match expectations (position-only update ≤ 60 bytes)
  - Fuzz-style tests with truncated buffers return errors, not crashes
  - Verify batch frame round-trip with 0, 1, 100 entities

### Files touched

- `include/StreamingCodec.h` (new)
- `src/StreamingCodec.cpp` (new)
- `tests/tactical_objects/Test_StreamingCodec.cpp` (new)
- `CMakeLists.txt` (add new sources)

---

## Step 3 — Version-Gated Delta Updates

**Goal:** Track which fields changed since the last version a subscriber received, and only serialize those fields. Avoids resending unchanged data (type, affiliation, milclass) on every position tick.

### Tasks

- [ ] Add per-component dirty flags to `ObjectStore`
  - Add `std::unordered_map<UUIDKey, uint16_t> dirty_masks_` to `ObjectStore`
  - Define component-to-bit mapping matching the `StreamingCodec` field mask
  - Every `set()` call on a `ComponentTable` also sets the corresponding bit in `dirty_masks_[key]`
  - Add `uint16_t getDirtyMask(const UUIDKey&) const` and `void clearDirtyMask(const UUIDKey&)`
  - Add `void clearAllDirtyMasks()`

- [ ] Add per-subscriber version tracking
  - In `TacticalObjectsRuntime`, maintain `std::unordered_map<SubscriptionHandle, std::unordered_map<UUIDKey, uint64_t>> subscriber_versions_`
  - On publish: compare entity's current version against subscriber's last-seen version
  - If subscriber has never seen the entity, send full snapshot (all bits set)
  - If version unchanged, skip
  - If version advanced, send only dirty fields accumulated since subscriber's last-seen version

- [ ] Integrate dirty masks into the publish cycle (from Step 1)
  - In `on_tick()`, for each dirty entity matched to an interest:
    - Compute the effective field mask from `getDirtyMask()`
    - Encode using `StreamingCodec::encodeEntityUpdate()` with that mask
    - Update subscriber's version record
  - After all subscribers processed, call `clearAllDirtyMasks()`

- [ ] Handle entity deletion
  - On delete, send a `message_type=0x02` frame (entity_delete) with just uuid + version
  - Remove entity from all subscriber version maps

- [ ] Handle new subscriber joining mid-stream
  - On `registerStreamSubscriber`, mark all entities matching the interest as needing a full snapshot (set all dirty bits or use version=0 sentinel)

- [ ] Unit tests
  - Position-only update produces field_mask with only bit 0 set
  - Affiliation change produces field_mask with only bit 2 set
  - New subscriber receives full snapshot on first tick
  - Unchanged entity between ticks produces no message
  - Deleted entity sends delete frame and is removed from version map

### Files touched

- `include/store/ObjectStore.h` / `src/store/ObjectStore.cpp`
- `include/store/ObjectComponents.h`
- `include/TacticalObjectsRuntime.h` / `src/TacticalObjectsRuntime.cpp`
- `tests/tactical_objects/Test_ObjectStore.cpp`
- `tests/tactical_objects/Test_TacticalObjectsRuntime.cpp`

---

## Step 4 — FlatBuffers Schema (Optional, Medium-Term)

**Goal:** If multi-language consumers are needed, introduce FlatBuffers for zero-copy deserialization with schema evolution. This step is optional and only warranted if consumers in Python, Java, or other languages need to read the stream.

### Tasks

- [ ] Define FlatBuffers schema `tactical_objects_stream.fbs`
  - `table EntityUpdate { uuid:[ubyte]; version:uint64; timestamp:float64; position:Position; velocity:Velocity; affiliation:Affiliation; ... }`
  - `table StreamFrame { tick_id:uint64; timestamp:float64; updates:[EntityUpdate]; deletes:[EntityDelete]; }`
  - Use FlatBuffers `optional` scalars for delta encoding

- [ ] Add FlatBuffers as a CMake dependency (header-only `flatc` codegen)
  - Add `FetchContent` for FlatBuffers
  - Add codegen step to `CMakeLists.txt`

- [ ] Implement `FlatBufferStreamingCodec`
  - Same interface as `StreamingCodec` but backed by FlatBuffers builders
  - Encode: build FlatBuffer, return `uint8_t*` + size (zero-copy producer)
  - Decode: use FlatBuffers accessors directly on the buffer (zero-copy consumer)

- [ ] Make codec selection configurable
  - Add `streaming_codec` parameter to component: `"binary"` (default) or `"flatbuffers"`
  - `TacticalObjectsComponent` instantiates the chosen codec in `on_configure()`

- [ ] Unit tests: `Test_FlatBufferStreamingCodec.cpp`
  - Same round-trip and size tests as Step 2
  - Verify zero-copy access (no intermediate allocations on decode)

### Files touched

- `schemas/tactical_objects_stream.fbs` (new)
- `include/FlatBufferStreamingCodec.h` (new)
- `src/FlatBufferStreamingCodec.cpp` (new)
- `tests/tactical_objects/Test_FlatBufferStreamingCodec.cpp` (new)
- `CMakeLists.txt`

---

## Step 5 — Batched Streaming Frames in `on_tick()`

**Goal:** Instead of one PCL message per entity update, batch all dirty entities for a tick into a single frame. This amortises transport overhead and is natural to produce in the existing `on_tick()` cadence.

### Tasks

- [ ] Define `StreamFrame` structure
  - `struct StreamFrame { uint64_t tick_id; double timestamp; std::vector<EntityUpdateFrame> updates; std::vector<UUIDKey> deletes; }`

- [ ] Implement frame assembly in `TacticalObjectsRuntime`
  - `StreamFrame assembleStreamFrame(const UUIDKey& interest_id, SubscriptionHandle subscriber)`
  - Iterate dirty entities, filter by interest, compute delta mask per entity, build frame
  - Return empty frame if no updates (caller skips publish)

- [ ] Implement batched publish in `TacticalObjectsComponent::on_tick()`
  - For each active interest with subscribers:
    - Call `assembleStreamFrame()`
    - If frame is non-empty, encode via `StreamingCodec::encodeBatchFrame()` and publish on the `entity_updates` publisher
  - Tag each published message with `interest_id` so consumers can demux

- [ ] Add configurable batching parameters
  - `max_entities_per_frame` (default: 500) — split large frames to bound latency
  - `streaming_tick_divisor` (default: 1) — publish every N ticks instead of every tick, for lower-priority interests

- [ ] Add frame sequence numbering
  - Each published frame carries a monotonic `sequence_id` per interest
  - Consumers can detect gaps (missed frames) and request a full snapshot resync

- [ ] Add snapshot resync service
  - `tactical_objects.resync` service: given an interest ID, returns a full snapshot frame of all matching entities
  - Used by consumers on startup or after detecting a sequence gap

- [ ] Unit tests
  - Single dirty entity produces a frame with one update
  - Multiple dirty entities produce a single frame with all updates
  - No dirty entities produces no publish call
  - `max_entities_per_frame` splits into multiple frames
  - Sequence IDs increment monotonically
  - Resync produces a full snapshot with all matching entities
  - `streaming_tick_divisor` skips intermediate ticks

- [ ] Integration test
  - Stand up `TacticalObjectsComponent` in a test PCL executor
  - Register an interest for Hostile Air entities in a bounding box
  - Inject 100 observations via `observation_ingress`
  - Verify published frames contain only matching entities
  - Verify wire size is within expected bounds (< 100 bytes/entity average)
  - Verify sequence continuity

### Files touched

- `include/TacticalObjectsRuntime.h` / `src/TacticalObjectsRuntime.cpp`
- `include/TacticalObjectsComponent.h` / `src/TacticalObjectsComponent.cpp`
- `include/StreamingCodec.h` / `src/StreamingCodec.cpp`
- `tests/tactical_objects/Test_TacticalObjectsRuntime.cpp`
- `tests/tactical_objects/Test_TacticalObjectsComponent.cpp`

---

## Dependency Graph

```
Step 1 (Interest-filtered push)
  │
  ├--> Step 2 (Binary streaming codec)
  │      │
  │      └--> Step 3 (Version-gated deltas)
  │             │
  │             ├--> Step 4 (FlatBuffers, optional)
  │             │
  │             └--> Step 5 (Batched frames in on_tick)
```

Steps 1 and 2 can proceed in parallel. Step 3 depends on both. Steps 4 and 5 depend on Step 3 and can proceed in parallel with each other.

## Design Constraints

- **C++14 maximum** — all code must compile under the C++14 standard
- **Single PCL thread** — all mutation and publication happens on the PCL executor thread; no additional synchronisation needed within the runtime
- **No new dependencies** for Steps 1–3 and 5. Step 4 adds FlatBuffers (header-only)
- **JSON codec preserved** — control-plane services (CRUD, query) continue to use `TacticalObjectsCodec` with `nlohmann::json`. The binary codec is additive, not a replacement
- **Backward compatible** — existing `events` publisher continues to work unchanged. The new `entity_updates` publisher is a separate channel
