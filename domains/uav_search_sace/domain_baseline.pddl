;;; ==========================================================================
;;; UAV Search Domain — SACE Stage 1: Baseline Operating Context
;;; ==========================================================================
;;; This is the baseline PDDL domain encoding the autonomous operating context.
;;; It is the starting point for all subsequent SACE stages.
;;;
;;; STRIPS-only — compatible with the mujin execution planner (LAPKT BRFS).
;;; ==========================================================================

(define (domain uav-search-sace)
  (:requirements :strips :typing)

  (:types
    location - object
    sector   - location
    robot    - object
  )

  (:predicates
    ;; --- Core operating predicates ---
    (at ?r - robot ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)

    ;; --- ODD boundary predicates (Stage 1) ---
    ;; These encode the Operational Design Domain limits.
    ;; Actions are gated on these so the planner cannot produce
    ;; plans that violate the defined operating context.
    (airspace-clear ?l - location)    ; airspace authorisation
    (weather-ok ?l - location)        ; within weather envelope
    (comms-available)                 ; C2 link active

    ;; --- Hazard predicates (Stage 2) ---
    ;; Used for reachability analysis / counterexample generation.
    (collision ?r - robot)
    (comms-lost)
  )

  ;; --- Nominal actions (Stage 1 capability set) ---

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (airspace-clear ?to)
      (weather-ok ?to)
      (comms-available)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  (:action search
    :parameters (?r - robot ?s - sector)
    :precondition (and
      (at ?r ?s)
      (comms-available)
    )
    :effect (searched ?s)
  )

  (:action classify
    :parameters (?r - robot ?s - sector)
    :precondition (and
      (at ?r ?s)
      (searched ?s)
      (comms-available)
    )
    :effect (classified ?s)
  )

  ;; --- Safe-state action (Stage 3 / Stage 7) ---
  ;; Always available regardless of comms — this is the fallback.

  (:action loiter-in-place
    :parameters (?r - robot ?l - location)
    :precondition (at ?r ?l)
    :effect ()  ; no state change; UAV holds position
  )
)
