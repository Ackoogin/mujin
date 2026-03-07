;;; ==========================================================================
;;; UAV Search Domain — SACE Stage 5: Design with degraded-mode actions
;;; ==========================================================================
;;; Extends the baseline domain with redundancy actions that operate under
;;; degraded conditions. These address the Stage 7 finding that safe-state
;;; recovery requires comms-independent actions.
;;;
;;; STRIPS-only — compatible with the mujin execution planner (LAPKT BRFS).
;;; ==========================================================================

(define (domain uav-search-sace-degraded)
  (:requirements :strips :typing)

  (:types
    location - object
    sector   - location
    robot    - object
  )

  (:predicates
    ;; --- Core ---
    (at ?r - robot ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)

    ;; --- ODD boundary ---
    (airspace-clear ?l - location)
    (weather-ok ?l - location)
    (comms-available)

    ;; --- Sensor status (Stage 5 redundancy) ---
    (gps-available)
    (inertial-nav-available)

    ;; --- Hazard / safe-state ---
    (collision ?r - robot)
    (comms-lost)
    (safe-state-reached ?r - robot)
  )

  ;; === Nominal actions (require comms) ===

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

  ;; === Degraded-mode actions (Stage 5 design improvement) ===

  ;; GPS-based return — works without comms but needs GPS
  (:action emergency-return-gps
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (airspace-clear ?to)
      (gps-available)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  ;; Inertial-only return — works without comms AND GPS
  (:action emergency-return-inertial
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (inertial-nav-available)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  ;; Loiter — always available
  (:action loiter-in-place
    :parameters (?r - robot ?l - location)
    :precondition (at ?r ?l)
    :effect (safe-state-reached ?r)
  )
)
