# Protocol-Agnostic Service Patterns Implementation Plan

## Scope
- In scope: packages/automtk-tactical-objects, automtk-tasks,
  automtk-objectives, automtk-sensors-component, automtk-guidance,
  automtk-fusion, automtk-sensor-products, automtk-msgs.
- Out of scope: packages/automtk-components (scheduled for deletion).

## Goals
1. Standardize request/response and streaming semantics across components.
2. Disambiguate client UPDATE commands vs component status updates.
3. Keep topic patterns protocol-agnostic (ROS2, gRPC, in-process pubsub).
4. Maintain backward compatibility where feasible with a deprecation window.

## Non-goals
- Implement ROS2 services/actions in this phase (only plan for it).
- Modify integration bridges unrelated to component services.

## Standard Message Model
### Envelope (existing)
- operation: CREATE, READ, UPDATE, DELETE
- message_id: unique per message (required)
- correlation_id: request message_id that this is responding to
- source_component: publisher identity
- timestamp, payload_type

### Requirement fields (existing)
- requirement_id: stable resource identifier
- status: PENDING, ACCEPTED, IN_PROGRESS, COMPLETED, FAILED, CANCELLED, REJECTED
- progress, result_json, failure_reason, parameters

### Direction and intent
Disambiguate by topic direction:
- Input topic: commands (CREATE, UPDATE, DELETE)
- Status topic: feedback and result (READ for progress, DELETE for final)

## Standard Topic Taxonomy (per component namespace)
### State Updates (pub/sub)
- ~/state (or component-specific noun, for example ~/track)
- CRUD encoded in envelope.operation and lifecycle fields

### Requirements (streaming)
- ~/requirement/input (client -> component)
  - CREATE: create requirement
  - UPDATE: update requirement parameters
  - DELETE: cancel requirement
- ~/requirement/status (component -> client)
  - READ: progress or interim status
  - DELETE: final result (COMPLETED/FAILED/CANCELLED/REJECTED)

### Queries (request/response)
- Prefer ROS2 services or gRPC unary calls
- If pub/sub only, use:
  - ~/query/input (READ request)
  - ~/query/response (READ response, correlation_id set)

## Correlation Rules
1. Every input request has a message_id.
2. Every status update uses correlation_id:
   - For CREATE: correlation_id = create.message_id
   - For UPDATE: correlation_id = update.message_id
   - For DELETE: correlation_id = cancel.message_id
3. requirement_id is stable across updates and status.
4. If server generates requirement_id, include it in the first status reply.

## Component-Level Implementation Plan
### 1) Tactical Objects
- Align requirement streaming to use ~/requirement/input and ~/requirement/status.
- Ensure status feedback uses READ for ACCEPTED/IN_PROGRESS, DELETE for final.
- Keep ~/state for CREATE/UPDATE/DELETE lifecycle events.
- Deprecate ~/requirement (publish) topic after compatibility window.

### 2) Tasks
- Already uses ~/requirement/input and ~/requirement/status.
- Ensure UPDATE from client is handled and correlated to status responses.
- Confirm READ feedback for ACCEPTED/IN_PROGRESS and DELETE for final.
- Add explicit handling of UPDATE in requirement parser if missing.

### 3) Objectives
- Already uses ~/objective/input and ~/objective/status.
- Mirror tasks requirement semantics (READ feedback, DELETE final).
- Add UPDATE handling for objective amendments and correlation.

### 4) Sensors Component
- Already uses ~/tasking/input and ~/tasking/status.
- Treat UPDATE on input as parameter change for active tasking.
- Ensure READ feedback uses correlation_id from the triggering request.
- Add CANCEL support via DELETE on input.

### 5) Guidance
- Already uses ~/trajectory/input and ~/trajectory/status.
- Add UPDATE handling for trajectory adjustments.
- Ensure READ feedback and DELETE final result semantics.

### 6) Fusion
- Split ~/fusion_requirement into:
  - ~/requirement/input (command)
  - ~/requirement/status (feedback)
- Publish solutions on ~/solution (state update, CREATE/UPDATE).
- Support CREATE/UPDATE/DELETE on input and correlate status.

### 7) Sensor Products
- State-only component: keep ~/track, ~/detection, ~/classification.
- No requirement streaming; queries can be service or request/response topics.

## ROS2 and gRPC Mapping
### ROS2
- Use Services for queries (READ request/response).
- Use Actions for requirements (goal/feedback/result).
- Keep pub/sub for state updates and capability/constraint topics.

### gRPC
- Unary RPCs for queries and create/update/cancel.
- Server streaming for requirement status.
- Bidirectional streaming optional for high-rate updates.

## Backward Compatibility Strategy
1. Publish to both old and new topics for one release.
2. Log deprecation warnings for old topics.
3. Remove old topics after adoption period.

## Testing Plan
1. Unit tests for requirement parsing and correlation_id usage.
2. Integration tests per component:
   - Send CREATE, verify READ feedback and DELETE final.
   - Send UPDATE, verify correlated feedback.
   - Send DELETE, verify CANCELLED final result.
3. ROS2 integration tests for actions/services where applicable.

## Open Questions
1. Should requirement_id be client-provided only, or can server generate?
2. Do we need a revision or version field for concurrent updates?
3. Should status feedback be throttled to avoid flooding subscribers?

