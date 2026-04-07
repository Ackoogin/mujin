"""ROS2 client wrapper for the AME Dev Environment.

Provides async service/action calls to WorldModelNode, PlannerNode, and
ExecutorNode.  Falls back gracefully when rclpy is not available (e.g. no
ROS2 installation), exposing ``available = False``.

All public methods are non-blocking: they submit requests and deliver results
via callback or by polling a Future-like object.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from ..models.events import WorldFact, WorldSnapshot

# Try to import rclpy; if unavailable we run in offline mode.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient as RclpyActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup

    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False


@dataclass
class PlanFeedback:
    """Streaming feedback from the planner action."""
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


class AmeRos2Client:
    """Wraps rclpy service/action calls for the dev environment UI.

    If rclpy is not importable, ``self.available`` is ``False`` and all
    methods become safe no-ops.
    """

    def __init__(self, node_name: str = "ame_devenv",
                 plan_action_name: str = "/planner_node/plan"):
        self.available = _HAS_RCLPY
        self._node_name = node_name
        self._plan_action_name = plan_action_name
        self._node: Any = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False
        self._pending_calls: deque[tuple[str, tuple[Any, ...]]] = deque()
        self._dispatch_timer: Any = None
        self.last_error: str = ""

        # Latest world state from subscription
        self._latest_snapshot: Optional[WorldSnapshot] = None
        self._snapshot_callbacks: list[Callable[[WorldSnapshot], None]] = []

        # Plan feedback queue
        self._plan_feedback_queue: deque[PlanFeedback] = deque(maxlen=100)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def latest_snapshot(self) -> Optional[WorldSnapshot]:
        return self._latest_snapshot

    def start(self) -> None:
        """Initialise rclpy and spin in a background thread."""
        if not self.available or self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="rclpy-spin", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Shutdown rclpy and join."""
        self._running = False
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self.available:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._connected = False

    def on_world_state(self, callback: Callable[[WorldSnapshot], None]) -> None:
        """Register a callback for world state updates."""
        self._snapshot_callbacks.append(callback)

    # -- Service calls (non-blocking, return via callback) -------------------

    def set_fact(
        self,
        key: str,
        value: bool,
        source: str = "devenv",
        callback: Optional[Callable[[bool, int], None]] = None,
    ) -> None:
        """Call ~/set_fact service. Callback receives (success, wm_version)."""
        if not self.available or not self._connected:
            return
        self._call_in_node_thread(
            "_do_set_fact", key, value, source, callback
        )

    def get_fact(
        self,
        key: str,
        callback: Optional[Callable[[bool, bool, int], None]] = None,
    ) -> None:
        """Call ~/get_fact service. Callback receives (found, value, wm_version)."""
        if not self.available or not self._connected:
            return
        self._call_in_node_thread("_do_get_fact", key, callback)

    def query_state(
        self,
        keys: Optional[list[str]] = None,
        callback: Optional[Callable[[WorldSnapshot], None]] = None,
    ) -> None:
        """Call ~/query_state service. Callback receives WorldSnapshot."""
        if not self.available or not self._connected:
            return
        self._call_in_node_thread("_do_query_state", keys or [], callback)

    def plan(
        self,
        goal_fluents: list[str],
        callback: Optional[Callable[[PlanResult], None]] = None,
    ) -> None:
        """Send /ame/plan action goal. Callback receives PlanResult."""
        if not self.available or not self._connected:
            return
        self._call_in_node_thread("_do_plan", goal_fluents, callback)

    def reload_domain(self, domain_path: str, problem_path: str) -> bool:
        """Reload PDDL domain and problem via the LoadDomain service.

        Reads the files locally and sends their content to the ROS2 load-domain
        services. Returns True on success.
        """
        if not self.available or not self._connected:
            self.last_error = "ROS2 client is not connected"
            return False

        try:
            domain_pddl = open(domain_path, "r", encoding="utf-8").read()
            problem_pddl = open(problem_path, "r", encoding="utf-8").read()
        except Exception as e:
            self.last_error = f"Failed to read PDDL files: {e}"
            print(f"[ROS2] {self.last_error}")
            return False

        load_clients = []
        if hasattr(self, "_planner_load_domain_cli") and self._planner_load_domain_cli is not None:
            load_clients.append(("planner", self._planner_load_domain_cli))
        if hasattr(self, "_load_domain_cli") and self._load_domain_cli is not None:
            load_clients.append(("world_model", self._load_domain_cli))
        if not load_clients:
            self.last_error = "LoadDomain service client is not available"
            return False

        domain_id = Path(domain_path).parent.name
        errors: list[str] = []
        success_count = 0
        for target_name, client in load_clients:
            if not client.wait_for_service(timeout_sec=2.0):
                continue

            req = self._LoadDomain.Request()
            req.domain_id = domain_id
            req.domain_pddl = domain_pddl
            req.problem_pddl = problem_pddl

            future = client.call_async(req)

            timeout = 5.0
            start = time.monotonic()
            while not future.done():
                if time.monotonic() - start > timeout:
                    errors.append(f"{target_name} load_domain timed out")
                    break
                time.sleep(0.05)

            if not future.done():
                continue

            result = future.result()
            if not result:
                errors.append(f"{target_name} load_domain returned no result")
                continue
            if not result.success:
                errors.append(
                    f"{target_name} load_domain failed: {result.error_msg or 'unknown error'}"
                )
                continue
            success_count += 1

        if errors:
            self.last_error = "; ".join(errors)
            print(f"[ROS2] {self.last_error}")
            return False
        if success_count == 0:
            self.last_error = "No load_domain service responded"
            print(f"[ROS2] {self.last_error}")
            return False

        self.last_error = ""
        self.query_state(callback=self._cache_snapshot)
        return True

    def drain_plan_feedback(self) -> list[PlanFeedback]:
        """Drain queued plan feedback for UI display."""
        items = []
        while self._plan_feedback_queue:
            try:
                items.append(self._plan_feedback_queue.popleft())
            except IndexError:
                break
        return items

    # -- Internal ------------------------------------------------------------

    def _call_in_node_thread(self, method_name: str, *args: Any) -> None:
        """Schedule a method call on the rclpy thread."""
        if not self._node:
            return
        self._pending_calls.append((method_name, args))

    def _drain_pending_calls(self) -> None:
        """Run queued UI requests on the ROS spin thread."""
        while self._pending_calls:
            method_name, args = self._pending_calls.popleft()
            getattr(self, method_name)(*args)

    def _run(self) -> None:
        """Background thread: init rclpy, create node, spin."""
        try:
            rclpy.init()
        except RuntimeError:
            pass  # already initialised

        self._node = rclpy.create_node(self._node_name)

        # Import service/action types (only available in a sourced workspace)
        try:
            from ame_ros2.srv import SetFact, GetFact, QueryState
            from ame_ros2.action import Plan
            from ame_ros2.msg import WorldState

            self._SetFact = SetFact
            self._GetFact = GetFact
            self._QueryState = QueryState
            self._Plan = Plan
            self._WorldState = WorldState

            # Create clients
            cb_group = ReentrantCallbackGroup()
            self._set_fact_cli = self._node.create_client(
                SetFact, "/world_model_node/set_fact", callback_group=cb_group
            )
            self._get_fact_cli = self._node.create_client(
                GetFact, "/world_model_node/get_fact", callback_group=cb_group
            )
            self._query_cli = self._node.create_client(
                QueryState, "/world_model_node/query_state", callback_group=cb_group
            )
            self._plan_cli = RclpyActionClient(
                self._node, Plan, self._plan_action_name, callback_group=cb_group
            )

            # LoadDomain is optional (requires ame_ros2 rebuild after adding the .srv)
            self._load_domain_cli = None
            self._planner_load_domain_cli = None
            try:
                from ame_ros2.srv import LoadDomain
                self._LoadDomain = LoadDomain
                self._load_domain_cli = self._node.create_client(
                    LoadDomain, "/world_model_node/load_domain", callback_group=cb_group
                )
                self._planner_load_domain_cli = self._node.create_client(
                    LoadDomain, "/planner_node/load_domain", callback_group=cb_group
                )
            except ImportError:
                pass

            # Subscribe to world state
            self._node.create_subscription(
                WorldState, "/world_state", self._on_world_state_msg, 10
            )
            self._dispatch_timer = self._node.create_timer(0.01, self._drain_pending_calls)

            self._connected = True
        except (ImportError, ModuleNotFoundError):
            # ame_ros2 messages not available — run without ROS2 services
            self._connected = False

        while self._running:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                break

    def _on_world_state_msg(self, msg: Any) -> None:
        """Handle incoming WorldState message."""
        facts = []
        for f in msg.facts:
            facts.append(WorldFact(key=f.key, value=f.value))
        snapshot = WorldSnapshot(
            wm_version=msg.wm_version,
            facts=facts,
            goal_fluents=list(msg.goal_fluents),
        )
        self._latest_snapshot = snapshot
        for cb in self._snapshot_callbacks:
            try:
                cb(snapshot)
            except Exception:
                pass

    def _cache_snapshot(self, snapshot: WorldSnapshot) -> None:
        """Store a refreshed snapshot and notify listeners."""
        self._latest_snapshot = snapshot
        for cb in self._snapshot_callbacks:
            try:
                cb(snapshot)
            except Exception:
                pass

    def _do_set_fact(
        self, key: str, value: bool, source: str, callback: Any
    ) -> None:
        if not self._set_fact_cli.wait_for_service(timeout_sec=2.0):
            return
        req = self._SetFact.Request()
        req.key = key
        req.value = value
        req.source = source
        future = self._set_fact_cli.call_async(req)
        future.add_done_callback(
            lambda f: callback(f.result().success, f.result().wm_version)
            if callback and f.result()
            else None
        )

    def _do_get_fact(self, key: str, callback: Any) -> None:
        if not self._get_fact_cli.wait_for_service(timeout_sec=2.0):
            return
        req = self._GetFact.Request()
        req.key = key
        future = self._get_fact_cli.call_async(req)
        future.add_done_callback(
            lambda f: callback(
                f.result().found, f.result().value, f.result().wm_version
            )
            if callback and f.result()
            else None
        )

    def _do_query_state(self, keys: list[str], callback: Any) -> None:
        if not self._query_cli.wait_for_service(timeout_sec=2.0):
            return
        req = self._QueryState.Request()
        req.keys = keys
        future = self._query_cli.call_async(req)

        def _on_done(f):
            if not callback or not f.result():
                return
            res = f.result()
            facts = [WorldFact(key=wf.key, value=wf.value) for wf in res.facts]
            snap = WorldSnapshot(
                wm_version=res.wm_version,
                facts=facts,
                goal_fluents=list(res.goal_fluents),
            )
            callback(snap)

        future.add_done_callback(_on_done)

    def _do_plan(self, goal_fluents: list[str], callback: Any) -> None:
        if not self._plan_cli.wait_for_server(timeout_sec=2.0):
            if callback:
                callback(PlanResult(success=False, error_msg="Plan action server unavailable"))
            return
        goal_msg = self._Plan.Goal()
        goal_msg.goal_fluents = goal_fluents
        goal_msg.replan = False
        if self._latest_snapshot is not None:
            self._latest_snapshot.goal_fluents = list(goal_fluents)
            for cb in self._snapshot_callbacks:
                try:
                    cb(self._latest_snapshot)
                except Exception:
                    pass

        def _on_feedback(feedback_msg):
            fb = feedback_msg.feedback
            self._plan_feedback_queue.append(PlanFeedback(
                nodes_expanded=fb.nodes_expanded,
                nodes_generated=fb.nodes_generated,
                elapsed_ms=fb.elapsed_ms,
                status_msg=fb.status_msg,
            ))

        def _on_result(future):
            if not callback:
                return
            try:
                result = future.result().result
                callback(PlanResult(
                    success=result.success,
                    plan_actions=list(result.plan_actions),
                    bt_xml=result.bt_xml,
                    solve_time_ms=result.solve_time_ms,
                    expanded=result.expanded,
                    generated=result.generated,
                    error_msg=result.error_msg,
                ))
            except Exception as e:
                callback(PlanResult(success=False, error_msg=str(e)))

        send_future = self._plan_cli.send_goal_async(
            goal_msg, feedback_callback=_on_feedback
        )
        send_future.add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(_on_result)
            if f.result() and f.result().accepted
            else (callback(PlanResult(success=False, error_msg="Goal rejected"))
                  if callback else None)
        )
