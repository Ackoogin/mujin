# Autonomy Backend Proto Alignment

This document describes the required changes to align the AME autonomy backend
(`subprojects/AME/include/ame/autonomy_backend.h`, `subprojects/AME/autonomy_backend.py`)
with the PYRAMID EntityActions CRUD proto definitions.

## Proto Files

| File | Purpose |
|------|---------|
| `pyramid/data_model/autonomy.proto` | Enums and entity messages with CRUD addressability |
| `pyramid/components/autonomy_backend/services/provided.proto` | 9 EntityActions services |

## EntityActions CRUD Structure

The proto defines 9 services following PYRAMID EntityActions conventions:

| Service | Flow | Operations | Replaces |
|---------|------|------------|----------|
| `Capabilities_Service` | out | Read | `describeCapabilities()` |
| `Session_Service` | inout | CRUD | `start()`, `readSnapshot()`, `step()`, `requestStop()` |
| `State_Service` | in | CUD | `pushState()` |
| `Intent_Service` | in | CUD | `pushIntent()` |
| `Command_Service` | out | Read | `pullCommands()` |
| `GoalDispatch_Service` | out | Read | `pullGoalDispatches()` |
| `DecisionRecord_Service` | out | Read | `pullDecisionRecords()` |
| `CommandResult_Service` | in | CUD | `pushCommandResult()` |
| `DispatchResult_Service` | in | CUD | `pushDispatchResult()` |

## Required Backend Changes

### 1. Add Entity Base to All Messages (REQUIRED)

All entity messages now include `pyramid.data_model.common.Entity base` for CRUD addressability:

```cpp
// Before (current)
struct SessionRequest {
  std::string session_id;
  MissionIntent intent;
  // ...
};

// After (CRUD-compliant)
struct Session {
  Entity base;           // id, source, update_time
  MissionIntent intent;
  // ...
};
```

**Affected types:**
- `SessionRequest` → `Session` (add `Entity base`)
- `StateUpdate` (add `Entity base`)
- `MissionIntent` (add `Entity base`)
- `AutonomyBackendCapabilities` → `Capabilities` (add `Entity base`)
- `ActionCommand` → `Command` (add `Entity base`)
- `GoalDispatch` (add `Entity base`)
- `DecisionRecord` (add `Entity base`)
- `CommandResult` (add `Entity base`)
- `DispatchResult` (add `Entity base`)
- `AutonomyBackendSnapshot` → `SessionSnapshot` (add `Entity base`)

---

### 2. Add `UNSPECIFIED = 0` to All Enums (REQUIRED)

Proto3 requires the zero value as default. Add explicit numbering:

```cpp
enum class AutonomyBackendState {
  UNSPECIFIED = 0,  // Add sentinel
  IDLE = 1,
  READY = 2,
  WAITING_FOR_RESULTS = 3,
  COMPLETE = 4,
  FAILED = 5,
  STOPPED = 6,
};
```

**Affected enums:** `AutonomyBackendState`, `CommandStatus`, `StopMode`, `FactAuthorityLevel`

---

### 3. Convert Pull Operations to Queryable Reads (SEMANTIC CHANGE)

**Current behavior:** `pullCommands()`, `pullGoalDispatches()`, `pullDecisionRecords()` are
**consumptive** - they drain internal queues and return items once.

**EntityActions behavior:** `ReadCommand()`, `ReadGoalDispatch()`, `ReadDecisionRecord()` are
**idempotent** - they query a persistent store and can return the same items multiple times.

**Required backend changes:**

```cpp
// Before: Queue-based (consumptive)
class IAutonomyBackend {
  virtual std::vector<ActionCommand> pullCommands() = 0;  // Drains queue
};

// After: Store-based (idempotent)
class IAutonomyBackend {
  // Query commands matching filter, optionally mark as "read"
  virtual std::vector<Command> readCommands(const Query& query) = 0;
  
  // Explicit acknowledgment to mark commands as processed
  virtual void acknowledgeCommands(const std::vector<Identifier>& ids) = 0;
};
```

**Options for implementation:**

1. **Persistent store with query**: Maintain commands/dispatches/records in a queryable store.
   Filter by ID, time range, or status. Support `one_shot` vs streaming semantics.

2. **Hybrid approach**: Keep queue internally but expose as queryable store externally.
   Track which items have been "read" separately from the queue.

3. **Add explicit Delete**: Use `DeleteCommand(Identifier)` to acknowledge processing
   (though this isn't in the current service definition since it's `out` flow).

---

### 4. Session Lifecycle Changes (INTERFACE CHANGE)

**Current interface:**
```cpp
void start(const SessionRequest& request);
void step();
void requestStop(StopMode mode);
AutonomyBackendSnapshot readSnapshot() const;
```

**EntityActions interface:**
```cpp
// Create session (returns session ID)
Identifier createSession(const Session& session);

// Read session snapshots (queryable)
std::vector<SessionSnapshot> readSession(const Query& query);

// Update session (trigger step)
void updateSession(const SessionStepRequest& request);

// Delete session (stop)
void deleteSession(const SessionStopRequest& request);
```

**Key differences:**
- `start()` → `createSession()` returns an `Identifier` (session ID)
- `step()` → `updateSession()` takes a `SessionStepRequest` with session ID
- `requestStop()` → `deleteSession()` takes a `SessionStopRequest` with session ID + mode
- `readSnapshot()` → `readSession()` uses `Query` filter and returns stream

---

### 5. State/Intent as First-Class Entities (SEMANTIC CHANGE)

**Current interface:**
```cpp
void pushState(const StateUpdate& update);   // Fire-and-forget
void pushIntent(const MissionIntent& intent); // Replaces intent
```

**EntityActions interface:**
```cpp
// State has Create/Update/Delete
Identifier createState(const StateUpdate& update);
void updateState(const StateUpdate& update);
void deleteState(const Identifier& id);

// Intent has Create/Update/Delete
Identifier createIntent(const MissionIntent& intent);
void updateIntent(const MissionIntent& intent);
void deleteIntent(const Identifier& id);
```

**Implementation notes:**
- State/Intent updates need IDs for addressability
- `createState` vs `updateState` distinction: create for new facts, update for changes
- `deleteState`/`deleteIntent` may not be commonly used but are required for full CRUD

---

### 6. Convert `request_fields` Map (REQUIRED)

**Current C++:**
```cpp
std::unordered_map<std::string, std::string> request_fields;
```

**Proto (for PYRAMID compatibility):**
```protobuf
repeated StringKeyValue request_fields = 7;
```

**Required change:**
```cpp
struct StringKeyValue { std::string key; std::string value; };
std::vector<StringKeyValue> request_fields;
```

---

### 7. Use Explicit Integer Types (RECOMMENDED)

```cpp
// Before
unsigned max_replans = 3;
unsigned replan_count = 0;

// After
uint32_t max_replans = 3;
uint32_t replan_count = 0;
```

---

## Summary: Current vs CRUD Interface

| Aspect | Current (Session-Based) | CRUD (EntityActions) |
|--------|------------------------|---------------------|
| **State model** | Stateful session with queues | Entity stores with CRUD ops |
| **Session lifecycle** | `start()` / `step()` / `requestStop()` | `Create` / `Update` / `Delete` |
| **Egress pattern** | `pull*()` drains queue | `Read*()` queries store |
| **Ingress pattern** | `push*()` fire-and-forget | `Create*()` / `Update*()` with IDs |
| **Addressability** | Implicit session state | Explicit `Entity base` with IDs |
| **Idempotency** | Pull is consumptive | Read is idempotent |

## Implementation Priorities

### Phase 1: Wire Compatibility (Minimal)
1. Add `UNSPECIFIED = 0` to enums
2. Add `Entity base` to messages (can be optional initially)
3. Convert `request_fields` to `vector<StringKeyValue>`
4. Keep existing `IAutonomyBackend` methods, adapt in transport layer

### Phase 2: Full CRUD Semantics
1. Implement queryable stores for Commands, GoalDispatches, DecisionRecords
2. Add session ID tracking and Query-based reads
3. Implement Create/Update/Delete distinction for State and Intent
4. Add explicit acknowledgment flow for processed items

### Phase 3: Multi-Service Split (Optional)
1. Split `IAutonomyBackend` into service-aligned interfaces
2. `ISessionService`, `IStateService`, `ICommandService`, etc.
3. Enables independent deployment and scaling

---

## File Locations

| Component | Path |
|-----------|------|
| C++ interface | `subprojects/AME/include/ame/autonomy_backend.h` |
| Python interface | `subprojects/AME/autonomy_backend.py` |
| Data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/autonomy.proto` |
| Services proto | `subprojects/PYRAMID/proto/pyramid/components/autonomy_backend/services/provided.proto` |
