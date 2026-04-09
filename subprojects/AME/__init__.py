"""AME Python package."""

try:
    from .autonomy_backend import (
        ActionCommand,
        AgentState,
        AmeAutonomyBackend,
        AutonomyBackend,
        AutonomyBackendCapabilities,
        AutonomyBackendSnapshot,
        AutonomyBackendState,
        CommandResult,
        CommandStatus,
        DecisionRecord,
        DispatchResult,
        FactAuthorityLevel,
        FactUpdate,
        GoalDispatch,
        MissionIntent,
        PolicyEnvelope,
        SessionRequest,
        StateUpdate,
        StopMode,
    )

    __all__ = [
        "ActionCommand",
        "AgentState",
        "AmeAutonomyBackend",
        "AutonomyBackend",
        "AutonomyBackendCapabilities",
        "AutonomyBackendSnapshot",
        "AutonomyBackendState",
        "CommandResult",
        "CommandStatus",
        "DecisionRecord",
        "DispatchResult",
        "FactAuthorityLevel",
        "FactUpdate",
        "GoalDispatch",
        "MissionIntent",
        "PolicyEnvelope",
        "SessionRequest",
        "StateUpdate",
        "StopMode",
    ]
except ImportError:
    # _ame_py binding may be stale or built for a different Python version.
    # Submodules (devenv, pcl_client, etc.) import _ame_py directly and are unaffected.
    __all__ = []
