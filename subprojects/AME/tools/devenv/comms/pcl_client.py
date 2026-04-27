"""PCL-direct client for the AME Dev Environment.

Provides the same interface as ros2_client but uses _ame_py bindings
directly, avoiding ROS2 dependency. All planning and world model
operations run in-process.
"""

from __future__ import annotations

import json
import threading
import time
import traceback
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
    """Direct PCL client -- same interface as AmeRos2Client.
    
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
        self._executor: Any = None

        # State tracking
        self._latest_snapshot: Optional[WorldSnapshot] = None
        self._snapshot_callbacks: list[Callable[[WorldSnapshot], None]] = []
        self._plan_feedback_queue: deque[PlanFeedback] = deque(maxlen=100)

        # BT event queue -- drained by the UI frame loop
        self._bt_event_queue: deque[str] = deque(maxlen=2000)
        # WM audit event queue -- drained by the UI frame loop
        self._wm_event_queue: deque[str] = deque(maxlen=2000)

        # Auto-tick control
        self._auto_tick_stop = threading.Event()
        self._auto_tick_thread: Optional[threading.Thread] = None
        self._tick_count: int = 0

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
            self._try_register_audit_callback()
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
        self.stop_execution()
        self._connected = False
        self._wm = None
        self._planner = None
        self._compiler = None
        self._registry = None
        self._executor = None

    def on_world_state(self, callback: Callable[[WorldSnapshot], None]) -> None:
        """Register a callback for world state updates."""
        self._snapshot_callbacks.append(callback)

    def reload_domain(self, domain_path: str, problem_path: str) -> bool:
        """Reload PDDL domain and problem files."""
        if not self.available or not self._connected:
            return False

        self.stop_execution()
        with self._lock:
            try:
                self._executor = None  # stale WM pointer; recreated on next load_bt
                self._wm = _ame_py.WorldModel()
                self._try_register_audit_callback()
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

    def _ensure_executor(self) -> None:
        """Create and wire up the executor if not already done (must hold lock)."""
        if self._executor is None:
            self._executor = _ame_py.ExecutorComponent()
            self._executor.set_inprocess_world_model(self._wm)
            self._executor.set_event_sink(self._on_bt_event)
            self._executor.configure()
            self._executor.activate()

    def _on_bt_event(self, event_json: str) -> None:
        """Event sink called by C++ executor on every BT node transition."""
        self._bt_event_queue.append(event_json)

    def drain_bt_events(self) -> list[str]:
        """Return and clear all queued BT event JSON strings."""
        items = []
        while self._bt_event_queue:
            try:
                items.append(self._bt_event_queue.popleft())
            except IndexError:
                break
        return items

    def _on_wm_audit(
        self,
        version: int,
        ts_us: int,
        fact: str,
        value: bool,
        source: str,
    ) -> None:
        """Audit callback fired by WorldModel on every fact change."""
        self._wm_event_queue.append(json.dumps({
            "wm_version": version,
            "ts_us": ts_us,
            "fact": fact,
            "value": value,
            "source": source,
        }))

    def drain_wm_events(self) -> list[str]:
        """Return and clear all queued WM audit event JSON strings."""
        items = []
        while self._wm_event_queue:
            try:
                items.append(self._wm_event_queue.popleft())
            except IndexError:
                break
        return items

    def load_bt(self, bt_xml: str) -> None:
        """Load a BT XML ready for ticking (manual or auto). Stops any running execution."""
        if not self.available or not self._connected:
            return
        self.stop_execution()
        with self._lock:
            self._ensure_executor()
            self._tick_count = 0
            self._executor.load_and_execute(bt_xml)

    def tick_once(self) -> str:
        """Tick the loaded BT once. Returns status name string."""
        if self._executor is None:
            return "IDLE"
        with self._lock:
            self._executor.tick_once()
            status = self._executor.last_status()
            self._tick_count += 1
        self._update_snapshot()
        return str(status).split(".")[-1]

    def start_auto_tick(
        self,
        interval_s: float = 0.05,
        done_callback: Optional[Callable[[bool, str], None]] = None,
    ) -> None:
        """Tick the loaded BT automatically in a background thread.

        Ticks continuously until stopped -- the tree re-evaluates each tick
        so it reacts to world-model fact changes.
        """
        self._auto_tick_stop.clear()

        def _auto() -> None:
            try:
                while not self._auto_tick_stop.is_set():
                    with self._lock:
                        if self._executor is None:
                            break
                        self._executor.tick_once()
                        self._tick_count += 1
                    self._auto_tick_stop.wait(interval_s)

                self._update_snapshot()
                if done_callback and self._executor:
                    raw = str(self._executor.last_status()).split(".")[-1]
                    done_callback(True, f"Stopped after {self._tick_count} ticks ({raw})")
            except Exception as e:
                traceback.print_exc()
                if done_callback:
                    done_callback(False, str(e))

        self._auto_tick_thread = threading.Thread(target=_auto, daemon=True)
        self._auto_tick_thread.start()

    def stop_execution(self) -> None:
        """Stop any running auto-tick thread and halt the BT."""
        self._auto_tick_stop.set()
        t = self._auto_tick_thread
        if t and t.is_alive():
            t.join(timeout=1.0)
        self._auto_tick_thread = None
        with self._lock:
            if self._executor is not None:
                try:
                    self._executor.halt_execution()
                except AttributeError:
                    pass  # older binding without halt_execution

    def is_executing(self) -> bool:
        return self._executor is not None and self._executor.is_executing()

    def last_bt_status(self) -> str:
        if self._executor is None:
            return "IDLE"
        return str(self._executor.last_status()).split(".")[-1]

    @property
    def tick_count(self) -> int:
        return self._tick_count

    def execute_bt(
        self,
        bt_xml: str,
        callback: Optional[Callable[[bool, str], None]] = None,
    ) -> None:
        """Execute a BT XML plan in one shot (used by Planning tab)."""
        if not self.available or not self._connected:
            if callback:
                callback(False, "Not connected")
            return

        def _do_execute() -> None:
            try:
                self.stop_execution()
                with self._lock:
                    self._ensure_executor()
                    self._tick_count = 0
                    self._executor.load_and_execute(bt_xml)

                max_ticks = 1000
                while self._tick_count < max_ticks:
                    with self._lock:
                        self._executor.tick_once()
                        self._tick_count += 1
                        raw = str(self._executor.last_status()).split(".")[-1]
                    if raw in ("SUCCESS", "FAILURE"):
                        break

                self._update_snapshot()
                raw = str(self._executor.last_status()).split(".")[-1]
                success = (raw == "SUCCESS")
                if callback:
                    callback(success, f"{raw} after {self._tick_count} ticks")

            except Exception as e:
                traceback.print_exc()
                if callback:
                    callback(False, str(e))

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

    def _try_register_audit_callback(self) -> None:
        """Register the WM audit callback if the binding supports it."""
        try:
            self._wm.set_audit_callback(self._on_wm_audit)

        except AttributeError:
            print("[PCL] set_audit_callback not available -- rebuild _ame_py for live WM events")

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
