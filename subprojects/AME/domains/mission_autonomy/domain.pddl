;;; ==========================================================================
;;; Mission Autonomy Domain
;;; ==========================================================================
;;; Models multi-agent mission planning with task reallocation contingency.
;;; Vehicle-level concerns (takeoff, landing, nav systems) are abstracted
;;; away — agents are simply "available" or not. This domain focuses on
;;; task execution and reallocation when an agent becomes unavailable.
;;;
;;; Tasks are pre-assigned in the problem file (representing the mission
;;; plan). Contingency: when an agent fails, its tasks are marked failed
;;; then reallocated to another available agent.
;;;
;;; NOTE: uses only positive preconditions (pure STRIPS). Degraded states
;;; are modelled with explicit predicates (agent-unavailable, sensor-degraded)
;;; rather than negated preconditions.
;;;
;;; STRIPS-only — compatible with the ame planner (LAPKT BRFS).
;;; ==========================================================================

(define (domain mission-autonomy)
  (:requirements :strips :typing)

  (:types
    location - object
    sector   - location
    agent    - object
  )

  (:predicates
    ;; --- Position ---
    (at ?a - agent ?l - location)

    ;; --- Agent status ---
    (agent-available ?a - agent)
    (agent-unavailable ?a - agent)
    (sensor-operational ?a - agent)
    (sensor-degraded ?a - agent)

    ;; --- Task state ---
    (searched ?s - sector)
    (classified ?s - sector)
    (task-assigned ?s - sector ?a - agent)
    (task-failed ?s - sector)

    ;; --- Mission state ---
    (comms-available)
  )

  ;; =====================================================================
  ;; NOMINAL ACTIONS
  ;; =====================================================================

  (:action move
    :parameters (?a - agent ?from - location ?to - location)
    :precondition (and
      (at ?a ?from)
      (agent-available ?a)
    )
    :effect (and
      (at ?a ?to)
      (not (at ?a ?from))
    )
  )

  (:action search
    :parameters (?a - agent ?s - sector)
    :precondition (and
      (at ?a ?s)
      (agent-available ?a)
      (sensor-operational ?a)
      (task-assigned ?s ?a)
    )
    :effect (searched ?s)
  )

  (:action classify
    :parameters (?a - agent ?s - sector)
    :precondition (and
      (at ?a ?s)
      (agent-available ?a)
      (sensor-operational ?a)
      (searched ?s)
      (task-assigned ?s ?a)
    )
    :effect (classified ?s)
  )

  ;; =====================================================================
  ;; CONTINGENCY ACTIONS — task reallocation
  ;; =====================================================================

  ;; An agent with degraded sensors withdraws from the mission.
  (:action withdraw-agent
    :parameters (?a - agent)
    :precondition (and
      (agent-available ?a)
      (sensor-degraded ?a)
    )
    :effect (and
      (agent-unavailable ?a)
      (not (agent-available ?a))
    )
  )

  ;; When an agent is unavailable, its assigned task is marked
  ;; failed and cleared, freeing it for reallocation.
  (:action mark-task-failed
    :parameters (?s - sector ?a - agent)
    :precondition (and
      (task-assigned ?s ?a)
      (agent-unavailable ?a)
    )
    :effect (and
      (task-failed ?s)
      (not (task-assigned ?s ?a))
    )
  )

  ;; A failed task is reassigned to an available agent with comms.
  (:action reallocate-task
    :parameters (?s - sector ?a - agent)
    :precondition (and
      (task-failed ?s)
      (agent-available ?a)
      (sensor-operational ?a)
      (comms-available)
    )
    :effect (and
      (task-assigned ?s ?a)
      (not (task-failed ?s))
    )
  )

  ;; Degraded search — sensor down but agent still available.
  (:action search-degraded
    :parameters (?a - agent ?s - sector)
    :precondition (and
      (at ?a ?s)
      (agent-available ?a)
      (sensor-degraded ?a)
      (task-assigned ?s ?a)
    )
    :effect (searched ?s)
  )
)
