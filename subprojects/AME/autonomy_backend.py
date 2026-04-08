from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List

from _ame_py import ActionRegistry, CurrentAmeBackendAdapter, PlanCompiler, Planner, WorldModel
import _ame_py


class FactAuthorityLevel(Enum):
    BELIEVED = "BELIEVED"
    CONFIRMED = "CONFIRMED"


class AutonomyBackendState(Enum):
    IDLE = "IDLE"
    READY = "READY"
    WAITING_FOR_RESULTS = "WAITING_FOR_RESULTS"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    STOPPED = "STOPPED"


class CommandStatus(Enum):
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED_TRANSIENT = "FAILED_TRANSIENT"
    FAILED_PERMANENT = "FAILED_PERMANENT"
    CANCELLED = "CANCELLED"


class StopMode(Enum):
    DRAIN = "DRAIN"
    IMMEDIATE = "IMMEDIATE"


@dataclass(slots=True)
class FactUpdate:
    key: str
    value: bool
    source: str = ""
    authority: FactAuthorityLevel = FactAuthorityLevel.CONFIRMED


@dataclass(slots=True)
class StateUpdate:
    fact_updates: List[FactUpdate] = field(default_factory=list)


@dataclass(slots=True)
class MissionIntent:
    goal_fluents: List[str] = field(default_factory=list)


@dataclass(slots=True)
class PolicyEnvelope:
    max_replans: int = 3
    enable_goal_dispatch: bool = False


@dataclass(slots=True)
class SessionRequest:
    session_id: str
    intent: MissionIntent
    policy: PolicyEnvelope = field(default_factory=PolicyEnvelope)


@dataclass(slots=True)
class AutonomyBackendCapabilities:
    backend_id: str
    supports_batch_planning: bool = True
    supports_external_command_dispatch: bool = True
    supports_replanning: bool = True


@dataclass(slots=True)
class ActionCommand:
    command_id: str
    action_name: str
    signature: str
    service_name: str
    operation: str
    request_fields: Dict[str, str] = field(default_factory=dict)


@dataclass(slots=True)
class GoalDispatch:
    dispatch_id: str
    agent_id: str
    goals: List[str] = field(default_factory=list)


@dataclass(slots=True)
class DecisionRecord:
    session_id: str
    backend_id: str
    world_version: int
    replan_count: int
    plan_success: bool
    solve_time_ms: float
    planned_action_signatures: List[str] = field(default_factory=list)
    compiled_bt_xml: str = ""


@dataclass(slots=True)
class CommandResult:
    command_id: str
    status: CommandStatus
    observed_updates: List[FactUpdate] = field(default_factory=list)
    source: str = ""


@dataclass(slots=True)
class DispatchResult:
    dispatch_id: str
    status: CommandStatus
    observed_updates: List[FactUpdate] = field(default_factory=list)
    source: str = ""


@dataclass(slots=True)
class AutonomyBackendSnapshot:
    session_id: str
    state: AutonomyBackendState
    world_version: int
    replan_count: int
    outstanding_commands: List[ActionCommand] = field(default_factory=list)
    outstanding_goal_dispatches: List[GoalDispatch] = field(default_factory=list)
    decision_history: List[DecisionRecord] = field(default_factory=list)


class AutonomyBackend(ABC):
    """Backend-neutral Python interface for a whole-system autonomy backend.

    Pure-Python or mixed-language backends should implement this surface directly.
    """

    @abstractmethod
    def describe_capabilities(self) -> AutonomyBackendCapabilities:
        raise NotImplementedError

    @abstractmethod
    def start(self, request: SessionRequest) -> None:
        raise NotImplementedError

    @abstractmethod
    def push_state(self, update: StateUpdate) -> None:
        raise NotImplementedError

    @abstractmethod
    def push_intent(self, intent: MissionIntent) -> None:
        raise NotImplementedError

    @abstractmethod
    def step(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def pull_commands(self) -> List[ActionCommand]:
        raise NotImplementedError

    @abstractmethod
    def pull_goal_dispatches(self) -> List[GoalDispatch]:
        raise NotImplementedError

    @abstractmethod
    def pull_decision_records(self) -> List[DecisionRecord]:
        raise NotImplementedError

    @abstractmethod
    def push_command_result(self, result: CommandResult) -> None:
        raise NotImplementedError

    @abstractmethod
    def push_dispatch_result(self, result: DispatchResult) -> None:
        raise NotImplementedError

    @abstractmethod
    def request_stop(self, mode: StopMode) -> None:
        raise NotImplementedError

    @abstractmethod
    def read_snapshot(self) -> AutonomyBackendSnapshot:
        raise NotImplementedError


def _to_cpp_fact_authority(authority: FactAuthorityLevel):
    return getattr(_ame_py.FactAuthorityLevel, authority.name)


def _to_cpp_state_update(update: StateUpdate):
    cpp_update = _ame_py.StateUpdate()
    cpp_items = []
    for item in update.fact_updates:
        cpp_item = _ame_py.FactUpdate()
        cpp_item.key = item.key
        cpp_item.value = item.value
        cpp_item.source = item.source
        cpp_item.authority = _to_cpp_fact_authority(item.authority)
        cpp_items.append(cpp_item)
    cpp_update.fact_updates = cpp_items
    return cpp_update


def _to_cpp_mission_intent(intent: MissionIntent):
    cpp_intent = _ame_py.MissionIntent()
    cpp_intent.goal_fluents = list(intent.goal_fluents)
    return cpp_intent


def _to_cpp_policy(policy: PolicyEnvelope):
    cpp_policy = _ame_py.PolicyEnvelope()
    cpp_policy.max_replans = policy.max_replans
    cpp_policy.enable_goal_dispatch = policy.enable_goal_dispatch
    return cpp_policy


def _to_cpp_session_request(request: SessionRequest):
    cpp_request = _ame_py.SessionRequest()
    cpp_request.session_id = request.session_id
    cpp_request.intent = _to_cpp_mission_intent(request.intent)
    cpp_request.policy = _to_cpp_policy(request.policy)
    return cpp_request


def _to_cpp_command_result(result: CommandResult):
    cpp_result = _ame_py.CommandResult()
    cpp_result.command_id = result.command_id
    cpp_result.status = getattr(_ame_py.CommandStatus, result.status.name)
    cpp_result.observed_updates = _to_cpp_state_update(
        StateUpdate(result.observed_updates)
    ).fact_updates
    cpp_result.source = result.source
    return cpp_result


def _to_cpp_dispatch_result(result: DispatchResult):
    cpp_result = _ame_py.DispatchResult()
    cpp_result.dispatch_id = result.dispatch_id
    cpp_result.status = getattr(_ame_py.CommandStatus, result.status.name)
    cpp_result.observed_updates = _to_cpp_state_update(
        StateUpdate(result.observed_updates)
    ).fact_updates
    cpp_result.source = result.source
    return cpp_result


def _from_cpp_capabilities(capabilities) -> AutonomyBackendCapabilities:
    return AutonomyBackendCapabilities(
        backend_id=capabilities.backend_id,
        supports_batch_planning=capabilities.supports_batch_planning,
        supports_external_command_dispatch=capabilities.supports_external_command_dispatch,
        supports_replanning=capabilities.supports_replanning,
    )


def _from_cpp_action_command(command) -> ActionCommand:
    return ActionCommand(
        command_id=command.command_id,
        action_name=command.action_name,
        signature=command.signature,
        service_name=command.service_name,
        operation=command.operation,
        request_fields=dict(command.request_fields),
    )


def _from_cpp_goal_dispatch(dispatch) -> GoalDispatch:
    return GoalDispatch(
        dispatch_id=dispatch.dispatch_id,
        agent_id=dispatch.agent_id,
        goals=list(dispatch.goals),
    )


def _from_cpp_decision_record(record) -> DecisionRecord:
    return DecisionRecord(
        session_id=record.session_id,
        backend_id=record.backend_id,
        world_version=record.world_version,
        replan_count=record.replan_count,
        plan_success=record.plan_success,
        solve_time_ms=record.solve_time_ms,
        planned_action_signatures=list(record.planned_action_signatures),
        compiled_bt_xml=record.compiled_bt_xml,
    )


def _from_cpp_snapshot(snapshot) -> AutonomyBackendSnapshot:
    return AutonomyBackendSnapshot(
        session_id=snapshot.session_id,
        state=AutonomyBackendState[snapshot.state.name],
        world_version=snapshot.world_version,
        replan_count=snapshot.replan_count,
        outstanding_commands=[
            _from_cpp_action_command(command)
            for command in snapshot.outstanding_commands
        ],
        outstanding_goal_dispatches=[
            _from_cpp_goal_dispatch(dispatch)
            for dispatch in snapshot.outstanding_goal_dispatches
        ],
        decision_history=[
            _from_cpp_decision_record(record)
            for record in snapshot.decision_history
        ],
    )


class AmeAutonomyBackend(AutonomyBackend):
    """Python wrapper around the current AME C++ backend adapter."""

    def __init__(
        self,
        world_model: WorldModel,
        action_registry: ActionRegistry,
        planner: Planner | None = None,
        plan_compiler: PlanCompiler | None = None,
    ) -> None:
        self._world_model = world_model
        self._action_registry = action_registry
        self._planner = planner or Planner()
        self._plan_compiler = plan_compiler or PlanCompiler()
        self._impl = CurrentAmeBackendAdapter(
            self._world_model,
            self._action_registry,
            self._planner,
            self._plan_compiler,
        )

    def describe_capabilities(self) -> AutonomyBackendCapabilities:
        return _from_cpp_capabilities(self._impl.describe_capabilities())

    def start(self, request: SessionRequest) -> None:
        self._impl.start(_to_cpp_session_request(request))

    def push_state(self, update: StateUpdate) -> None:
        self._impl.push_state(_to_cpp_state_update(update))

    def push_intent(self, intent: MissionIntent) -> None:
        self._impl.push_intent(_to_cpp_mission_intent(intent))

    def step(self) -> None:
        self._impl.step()

    def pull_commands(self) -> List[ActionCommand]:
        return [_from_cpp_action_command(c) for c in self._impl.pull_commands()]

    def pull_goal_dispatches(self) -> List[GoalDispatch]:
        return [_from_cpp_goal_dispatch(d) for d in self._impl.pull_goal_dispatches()]

    def pull_decision_records(self) -> List[DecisionRecord]:
        return [
            _from_cpp_decision_record(r)
            for r in self._impl.pull_decision_records()
        ]

    def push_command_result(self, result: CommandResult) -> None:
        self._impl.push_command_result(_to_cpp_command_result(result))

    def push_dispatch_result(self, result: DispatchResult) -> None:
        self._impl.push_dispatch_result(_to_cpp_dispatch_result(result))

    def request_stop(self, mode: StopMode) -> None:
        self._impl.request_stop(getattr(_ame_py.StopMode, mode.name))

    def read_snapshot(self) -> AutonomyBackendSnapshot:
        return _from_cpp_snapshot(self._impl.read_snapshot())
