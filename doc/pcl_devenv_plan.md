# PCL-Only DevEnv Deployment Plan

## Objective

Enable `tools/devenv` to run without ROS2 by providing Python bindings directly to the PCL-level components (`WorldModelComponent`, `PlannerComponent`, `ExecutorComponent`).

---

## Current Architecture

```
┌─────────────────┐      ROS2 Topics/Services      ┌─────────────────┐
│  tools/devenv   │ ◄──────────────────────────────► │  ROS2 Nodes     │
│  (Python UI)    │   rclpy                         │  (C++ lifecycle)│
│                 │                                  │                 │
│  ros2_client.py │                                  │  WorldModelNode │
│                 │                                  │  PlannerNode    │
│                 │                                  │  ExecutorNode   │
└─────────────────┘                                  └─────────────────┘
```

**Problem**: Requires full ROS2 stack, which has issues on Windows.

---

## Proposed Architecture

```
┌─────────────────┐                                  ┌─────────────────┐
│  tools/devenv   │                                  │  ame_core       │
│  (Python UI)    │                                  │  (C++ PCL)      │
│                 │      pybind11                    │                 │
│  pcl_client.py  │ ◄──────────────────────────────► │  WorldModel-    │
│      or         │   _ame_py extension              │  Component      │
│  ros2_client.py │                                  │  Planner-       │
│                 │                                  │  Component      │
│                 │                                  │  Executor-      │
│                 │                                  │  Component      │
└─────────────────┘                                  └─────────────────┘
```

---

## Implementation Steps

### Step 1: Python Bindings (`src/ame/python/`)

Create pybind11 bindings for core components:

```cpp
// src/ame/python/ame_py.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ame/world_model.h>
#include <ame/planner.h>
#include <ame/plan_compiler.h>
#include <ame/action_registry.h>
#include <ame/pddl_parser.h>

namespace py = pybind11;

PYBIND11_MODULE(_ame_py, m) {
    // WorldModel bindings
    py::class_<ame::WorldModel>(m, "WorldModel")
        .def(py::init<>())
        .def("set_fact", &ame::WorldModel::setFact)
        .def("get_fact", py::overload_cast<const std::string&>(&ame::WorldModel::getFact, py::const_))
        .def("version", &ame::WorldModel::version)
        .def("set_goal", &ame::WorldModel::setGoal)
        .def("goal_fluent_ids", &ame::WorldModel::goalFluentIds)
        .def("fluent_name", &ame::WorldModel::fluentName)
        .def("num_fluents", &ame::WorldModel::numFluents)
        // Agent management
        .def("register_agent", &ame::WorldModel::registerAgent)
        .def("agent_ids", &ame::WorldModel::agentIds)
        .def("available_agent_ids", &ame::WorldModel::availableAgentIds);

    // Planner bindings
    py::class_<ame::PlanResult>(m, "PlanResult")
        .def_readonly("success", &ame::PlanResult::success)
        .def_readonly("solve_time_ms", &ame::PlanResult::solve_time_ms)
        .def_readonly("expanded", &ame::PlanResult::expanded)
        .def_readonly("generated", &ame::PlanResult::generated)
        .def_readonly("cost", &ame::PlanResult::cost);

    py::class_<ame::Planner>(m, "Planner")
        .def(py::init<>())
        .def("solve", &ame::Planner::solve);

    // PlanCompiler bindings
    py::class_<ame::PlanCompiler>(m, "PlanCompiler")
        .def(py::init<>())
        .def("compile", py::overload_cast<const std::vector<ame::PlanStep>&,
             const ame::WorldModel&, const ame::ActionRegistry&>
             (&ame::PlanCompiler::compile, py::const_));

    // PDDL Parser
    m.def("load_domain", &ame::loadDomain);
    m.def("load_problem", &ame::loadProblem);
}
```

### Step 2: CMake Target

```cmake
# src/ame/CMakeLists.txt (additions)

# Optional: Python bindings
option(AME_BUILD_PYTHON "Build Python bindings" OFF)

if(AME_BUILD_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)

    pybind11_add_module(_ame_py python/ame_py.cpp)
    target_link_libraries(_ame_py PRIVATE ame_core)
    
    # Install to python site-packages or local directory
    install(TARGETS _ame_py DESTINATION ${CMAKE_INSTALL_PREFIX}/python)
endif()
```

### Step 3: PCL Client (`tools/devenv/comms/pcl_client.py`)

```python
"""PCL-direct client for AME Dev Environment.

Provides the same interface as ros2_client but uses _ame_py bindings
directly, avoiding ROS2 dependency.
"""

import threading
from typing import Callable, Optional
from ..models.events import WorldFact, WorldSnapshot

try:
    import _ame_py
    _HAS_AME_PY = True
except ImportError:
    _HAS_AME_PY = False


class AmePclClient:
    """Direct PCL client - same interface as AmeRos2Client."""

    def __init__(self):
        self.available = _HAS_AME_PY
        self._wm: Optional[_ame_py.WorldModel] = None
        self._planner: Optional[_ame_py.Planner] = None
        self._compiler: Optional[_ame_py.PlanCompiler] = None
        self._lock = threading.Lock()
        self._connected = False
        self._snapshot_callbacks = []

    def start(self, domain_path: str = "", problem_path: str = "") -> None:
        if not self.available:
            return
        self._wm = _ame_py.WorldModel()
        self._planner = _ame_py.Planner()
        self._compiler = _ame_py.PlanCompiler()
        
        if domain_path and problem_path:
            _ame_py.load_domain(self._wm, domain_path)
            _ame_py.load_problem(self._wm, problem_path)
        
        self._connected = True

    def stop(self) -> None:
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def set_fact(self, key: str, value: bool, source: str = "devenv",
                 callback: Optional[Callable] = None) -> None:
        if not self._connected:
            return
        with self._lock:
            self._wm.set_fact(key, value)
            ver = self._wm.version()
        if callback:
            callback(True, ver)
        self._notify_snapshot()

    def get_fact(self, key: str, callback: Optional[Callable] = None) -> None:
        if not self._connected:
            return
        with self._lock:
            val = self._wm.get_fact(key)
            ver = self._wm.version()
        if callback:
            callback(True, val, ver)

    def plan(self, goal_fluents: list[str],
             callback: Optional[Callable] = None) -> None:
        if not self._connected:
            return
        # Run planning in background thread
        def _do_plan():
            with self._lock:
                self._wm.set_goal(goal_fluents)
                result = self._planner.solve(self._wm)
                # ... compile BT XML, call callback
        threading.Thread(target=_do_plan, daemon=True).start()
```

### Step 4: Update DevEnv Config

```python
# tools/devenv/config.py

@dataclass
class ConnectionConfig:
    backend: str = "ros2"  # "ros2" or "pcl"
    # ... existing fields
```

### Step 5: Backend Selection in main.py

```python
# tools/devenv/main.py

parser.add_argument(
    "--backend",
    choices=["ros2", "pcl"],
    default="ros2",
    help="Communication backend (default: ros2)",
)

# In main():
if args.backend == "pcl":
    from .comms.pcl_client import AmePclClient
    client = AmePclClient()
else:
    from .comms.ros2_client import AmeRos2Client
    client = AmeRos2Client()
```

### Step 6: Update Batch Scripts

```batch
@rem start_devenv_pcl.bat
@echo off
set PYTHONPATH=%~dp0..\..\build\install\python;%PYTHONPATH%
python -m tools.devenv --backend pcl
```

---

## File Summary

| File | Action | Purpose |
|------|--------|---------|
| `src/ame/python/ame_py.cpp` | New | pybind11 bindings |
| `src/ame/CMakeLists.txt` | Modify | Add Python target |
| `tools/devenv/comms/pcl_client.py` | New | Direct PCL client |
| `tools/devenv/config.py` | Modify | Add backend option |
| `tools/devenv/main.py` | Modify | Backend selection |
| `tools/devenv/start_devenv_pcl.bat` | New | PCL-mode launcher |

---

## Dependencies

- **pybind11**: Add via FetchContent or find_package
- **Python 3.8+**: For type hints and modern features

---

## Benefits

1. **No ROS2 required** — runs on vanilla Windows
2. **Same UI** — devenv unchanged from user perspective  
3. **Lower latency** — direct function calls vs IPC
4. **Simpler debugging** — single process, no message serialization

---

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| Thread safety | Use mutex in PCL client, components already thread-safe |
| BT execution | ExecutorComponent already ROS-agnostic |
| Missing features | Maintain ros2_client for full deployment |

---

## Implementation Order

1. ✅ Create plan document (this file)
2. Add pybind11 to CMake
3. Create minimal bindings (`WorldModel`, `Planner`)
4. Create `pcl_client.py` with same interface
5. Add `--backend pcl` to devenv
6. Test end-to-end
