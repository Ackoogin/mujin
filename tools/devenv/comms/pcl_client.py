"""PCL-direct client for the AME Dev Environment.

Provides the same interface as ros2_client but uses _ame_py bindings
directly, avoiding ROS2 dependency. All planning and world model
operations run in-process.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, List

from ..models.events import WorldFact, WorldSnapshot

# Try to import the native Python bindings
try:
    import _ame_py
    _HAS_AME_PY = True
except ImportError:
    _HAS_AME_PY = False


@dataclass
class PlanFeedback:
    """Streaming feedback from the planner."""
    nodes_expanded: int = 0
    nodes_generated: int = 0
    elapsed_ms: float = 0.0
    status_msg: str = ""


@dataclass
class PlanResult:
    """Result of a planning request."""
    success: bool = False
    plan_actions: list[str] = field(default_factory=list)
    bt_xml: str = ""
    solve_time_ms: float = 0.0
    expanded: int = 0
    generated: int = 0
    error_msg: str = ""


class AmePclClient:
    """Direct PCL client — same interface as AmeRos2Client.
    
    Uses _ame_py native bindings to call WorldModel, Planner, and
    PlanCompiler directly without ROS2 middleware.
    """

    def __init__(self, node_name: str = "ame_devenv"):
        self.available = _HAS_AME_PY
        self._node_name = node_name
        self._lock = threading.RLock()  # Reentrant lock to avoid deadlock
        self._connected = False

        # Core components
        self._wm: Any = None
        self._planner: Any = None
        self._compiler: Any = None
        self._registry: Any = None

        # State tracking
        self._latest_snapshot: Optional[WorldSnapshot] = None
        self._snapshot_callbacks: list[Callable[[WorldSnapshot], None]] = []
        self._plan_feedback_queue: deque[PlanFeedback] = deque(maxlen=100)

        # Domain/problem paths for initialization
        self._domain_path: str = ""
        self._problem_path: str = ""

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def latest_snapshot(self) -> Optional[WorldSnapshot]:
        return self._latest_snapshot

    def start(self, domain_path: str = "", problem_path: str = "") -> None:
        """Initialize components and optionally load PDDL files."""
        if not self.available:
            return

        self._domain_path = domain_path
        self._problem_path = problem_path

        with self._lock:
            self._wm = _ame_py.WorldModel()
            self._planner = _ame_py.Planner()
            self._compiler = _ame_py.PlanCompiler()
            self._registry = _ame_py.ActionRegistry()

            if domain_path and problem_path:
                try:
                    _ame_py.parse_pddl(self._wm, domain_path, problem_path)
                except Exception as e:
                    print(f"[PCL] Failed to load PDDL: {e}")

            self._connected = True
            self._update_snapshot()

    def stop(self) -> None:
        """Shutdown the client."""
        self._connected = False
        self._wm = None
        self._planner = None
        self._compiler = None
        self._registry = None

    def on_world_state(self, callback: Callable[[WorldSnapshot], None]) -> None:
        """Register a callback for world state updates."""
        self._snapshot_callbacks.append(callback)

    def reload_domain(self, domain_path: str, problem_path: str) -> bool:
        """Reload PDDL domain and problem files."""
        if not self.available or not self._connected:
            return False

        with self._lock:
            try:
                # Create fresh world model
                self._wm = _ame_py.WorldModel()
                _ame_py.parse_pddl(self._wm, domain_path, problem_path)
                self._domain_path = domain_path
                self._problem_path = problem_path
                self._update_snapshot()
                return True
            except Exception as e:
                print(f"[PCL] Failed to reload PDDL: {e}")
                return False

    # -- Fact operations (same interface as ros2_client) -----------------------

    def set_fact(
        self,
        key: str,
        value: bool,
        source: str = "devenv",
        callback: Optional[Callable[[bool, int], None]] = None,
    ) -> None:
        """Set a fact in the world model."""
        if not self.available or not self._connected:
            return

        with self._lock:
            try:
                self._wm.set_fact(key, value)
                ver = self._wm.version()
                success = True
            except Exception:
                success = False
                ver = 0

        if callback:
            callback(success, ver)

        if success:
            self._update_snapshot()

    def get_fact(
        self,
        key: str,
        callback: Optional[Callable[[bool, bool, int], None]] = None,
    ) -> None:
        """Get a fact from the world model."""
        if not self.available or not self._connected:
            return

        with self._lock:
            try:
                val = self._wm.get_fact(key)
                ver = self._wm.version()
                found = True
            except Exception:
                found = False
                val = False
                ver = 0

        if callback:
            callback(found, val, ver)

    def query_state(
        self,
        keys: Optional[list[str]] = None,
        callback: Optional[Callable[[WorldSnapshot], None]] = None,
    ) -> None:
        """Query the world state (optionally filtered by keys)."""
        if not self.available or not self._connected:
            return

        with self._lock:
            snapshot = self._build_snapshot()

        if keys:
            # Filter to requested keys
            filtered_facts = [f for f in snapshot.facts if f.key in keys]
            snapshot = WorldSnapshot(
                wm_version=snapshot.wm_version,
                facts=filtered_facts,
                goal_fluents=snapshot.goal_fluents,
            )

        if callback:
            callback(snapshot)

    def plan(
        self,
        goal_fluents: list[str],
        callback: Optional[Callable[[PlanResult], None]] = None,
    ) -> None:
        """Run the planner on the given goals."""
        if not self.available or not self._connected:
            return

        # Run planning in a background thread to avoid blocking UI
        def _do_plan():
            result = PlanResult()
            try:
                with self._lock:
                    self._wm.set_goal(goal_fluents)
                    plan_result = self._planner.solve(self._wm)

                    result.success = plan_result.success
                    result.solve_time_ms = plan_result.solve_time_ms
                    result.expanded = plan_result.expanded
                    result.generated = plan_result.generated

                    if plan_result.success:
                        # Extract action signatures
                        for step in plan_result.steps:
                            sig = _ame_py.get_action_signature(
                                self._wm, step.action_index
                            )
                            result.plan_actions.append(sig)

                        # Compile to BT XML
                        result.bt_xml = self._compiler.compile(
                            plan_result.steps, self._wm, self._registry
                        )
                    else:
                        result.error_msg = plan_result.error_msg or "Planning failed"

            except Exception as e:
                result.success = False
                result.error_msg = str(e)

            if callback:
                callback(result)

        threading.Thread(target=_do_plan, daemon=True).start()

    def drain_plan_feedback(self) -> list[PlanFeedback]:
        """Drain queued plan feedback for UI display."""
        items = []
        while self._plan_feedback_queue:
            try:
                items.append(self._plan_feedback_queue.popleft())
            except IndexError:
                break
        return items

    # -- Execution operations --------------------------------------------------

    def execute_bt(
        self,
        bt_xml: str,
        callback: Optional[Callable[[bool, str], None]] = None,
    ) -> None:
        """Execute a BT XML plan."""
        if not self.available or not self._connected:
            if callback:
                callback(False, "Not connected")
            return

        def _do_execute():
            try:
                # Create executor if needed
                if not hasattr(self, '_executor') or self._executor is None:
                    self._executor = _ame_py.ExecutorComponent()
                    self._executor.set_inprocess_world_model(self._wm)
                    self._executor.configure()
                    self._executor.activate()

                # Load and execute the BT
                self._executor.load_and_execute(bt_xml)
                
                # Tick until completion
                max_ticks = 1000
                tick_count = 0
                while self._executor.is_executing() and tick_count < max_ticks:
                    self._executor.tick_once()
                    tick_count += 1
                
                status = self._executor.last_status()
                if status == _ame_py.NodeStatus.SUCCESS:
                    if callback:
                        callback(True, f"SUCCESS after {tick_count} ticks")
                elif status == _ame_py.NodeStatus.FAILURE:
                    if callback:
                        callback(False, f"FAILURE after {tick_count} ticks")
                else:
                    if callback:
                        callback(False, f"Stopped at {status} after {tick_count} ticks")
                
                # Update world model snapshot after execution
                self._update_snapshot()
                
            except Exception as e:
                if callback:
                    callback(False, str(e))

        # Run in background thread
        threading.Thread(target=_do_execute, daemon=True).start()

    # -- Agent operations ------------------------------------------------------

    def register_agent(self, agent_id: str, agent_type: str) -> None:
        """Register an agent in the world model."""
        if not self.available or not self._connected:
            return

        with self._lock:
            self._wm.register_agent(agent_id, agent_type)

    def get_agent_ids(self) -> list[str]:
        """Get all registered agent IDs."""
        if not self.available or not self._connected:
            return []

        with self._lock:
            return self._wm.agent_ids()

    def get_available_agent_ids(self) -> list[str]:
        """Get available agent IDs."""
        if not self.available or not self._connected:
            return []

        with self._lock:
            return self._wm.available_agent_ids()

    # -- Internal --------------------------------------------------------------

    def _build_snapshot(self) -> WorldSnapshot:
        """Build a WorldSnapshot from current state (must hold lock)."""
        facts = []
        try:
            true_facts = self._wm.all_true_facts()
            for key in true_facts:
                facts.append(WorldFact(key=key, value=True))
        except Exception:
            pass

        goal_fluents = []
        try:
            goal_ids = self._wm.goal_fluent_ids()
            for gid in goal_ids:
                goal_fluents.append(self._wm.fluent_name(gid))
        except Exception:
            pass

        return WorldSnapshot(
            wm_version=self._wm.version() if self._wm else 0,
            facts=facts,
            goal_fluents=goal_fluents,
        )

    def _update_snapshot(self) -> None:
        """Update cached snapshot and notify callbacks."""
        with self._lock:
            self._latest_snapshot = self._build_snapshot()

        for cb in self._snapshot_callbacks:
            try:
                cb(self._latest_snapshot)
            except Exception:
                pass
