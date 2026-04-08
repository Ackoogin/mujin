from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

import _ame_py

from _ame_py import (
    ActionCommand,
    ActionRegistry,
    AutonomyBackendCapabilities,
    AutonomyBackendSnapshot,
    CommandResult,
    CurrentAmeBackendAdapter,
    DecisionRecord,
    MissionIntent,
    Planner,
    PlanCompiler,
    SessionRequest,
    StateUpdate,
    StopMode,
    WorldModel,
)


class AutonomyBackend(ABC):
    """Backend-neutral Python interface for a whole-system autonomy backend."""

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
    def pull_decision_records(self) -> List[DecisionRecord]:
        raise NotImplementedError

    @abstractmethod
    def push_command_result(self, result: CommandResult) -> None:
        raise NotImplementedError

    @abstractmethod
    def request_stop(self, mode: StopMode) -> None:
        raise NotImplementedError

    @abstractmethod
    def read_snapshot(self) -> AutonomyBackendSnapshot:
        raise NotImplementedError


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
        return self._impl.describe_capabilities()

    def start(self, request: SessionRequest) -> None:
        self._impl.start(request)

    def push_state(self, update: StateUpdate) -> None:
        self._impl.push_state(update)

    def push_intent(self, intent: MissionIntent) -> None:
        self._impl.push_intent(intent)

    def step(self) -> None:
        self._impl.step()

    def pull_commands(self) -> List[ActionCommand]:
        return list(self._impl.pull_commands())

    def pull_decision_records(self) -> List[DecisionRecord]:
        return list(self._impl.pull_decision_records())

    def push_command_result(self, result: CommandResult) -> None:
        self._impl.push_command_result(result)

    def request_stop(self, mode: StopMode) -> None:
        self._impl.request_stop(mode)

    def read_snapshot(self) -> AutonomyBackendSnapshot:
        return self._impl.read_snapshot()
