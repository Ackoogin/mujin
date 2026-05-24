;;; ==========================================================================
;;; Vehicle Autonomy Domain
;;; ==========================================================================
;;; Models the flight lifecycle of a single UAV with contingency actions.
;;; The mission itself is abstract — represented as a single predicate that
;;; gets set at the mission location. This domain focuses on the vehicle
;;; getting safely from ground to mission area and back, with recovery
;;; actions for every failure mode at every phase.
;;;
;;; Contingency actions are modelled as alternative action sets with
;;; relaxed preconditions (per SACE Stage 5 design assurance).  A single
;;; domain carries both nominal and contingency actions — the planner
;;; selects the appropriate path based on which predicates are present.
;;;
;;; Redundancy ladder (most capable → last resort):
;;;   nominal-move > emergency-return-nav > emergency-return-gps
;;;                > emergency-return-inertial > emergency-land
;;;
;;; Critical mission ditch ladder (post-mission, expendable vehicle):
;;;   fly-to-ditch-gps + hard-land-ditch > fly-to-ditch-inertial
;;;   + hard-land-ditch > declare-ditch-emergency + terminal-ditch
;;;
;;; STRIPS-only — compatible with the ame planner (LAPKT BRFS).
;;; ==========================================================================

(define (domain vehicle-autonomy)
  (:requirements :strips :typing)

  (:types
    location - object
    base     - location
    waypoint - location
    robot    - object
  )

  (:predicates
    ;; --- Position & flight state ---
    (at ?r - robot ?l - location)
    (on-ground ?r - robot)
    (airborne ?r - robot)
    (preflight-done ?r - robot)

    ;; --- Mission (abstract) ---
    (mission-objective ?l - location)
    (mission-complete ?r - robot)

    ;; --- System health ---
    (nav-operational)
    (engines-operational)
    (comms-available)
    (gps-available)
    (inertial-nav-available)

    ;; --- ODD boundary ---
    (airspace-clear ?l - location)
    (weather-ok ?l - location)

    ;; --- Mission priority ---
    (mission-priority-critical)

    ;; --- Ditch / terminal destruction ---
    (ditch-zone-designated ?l - location)
    (vehicle-ditched ?r - robot)

    ;; --- Contingency state ---
    (mission-aborted ?r - robot)
    (emergency-declared ?r - robot)
    (safe-state ?r - robot)
    (landed-safe ?r - robot)
  )

  ;; =====================================================================
  ;; NOMINAL ACTIONS
  ;; =====================================================================

  (:action preflight-check
    :parameters (?r - robot ?l - base)
    :precondition (and
      (at ?r ?l)
      (on-ground ?r)
      (nav-operational)
      (engines-operational)
      (comms-available)
    )
    :effect (preflight-done ?r)
  )

  (:action takeoff
    :parameters (?r - robot ?l - base)
    :precondition (and
      (at ?r ?l)
      (on-ground ?r)
      (preflight-done ?r)
      (nav-operational)
      (engines-operational)
      (airspace-clear ?l)
      (weather-ok ?l)
      (comms-available)
    )
    :effect (and
      (airborne ?r)
      (not (on-ground ?r))
    )
  )

  (:action fly
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
      (nav-operational)
      (engines-operational)
      (airspace-clear ?to)
      (weather-ok ?to)
      (comms-available)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  (:action execute-mission
    :parameters (?r - robot ?l - waypoint)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
      (comms-available)
      (mission-objective ?l)
    )
    :effect (mission-complete ?r)
  )

  (:action land
    :parameters (?r - robot ?l - base)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
      (nav-operational)
      (engines-operational)
      (weather-ok ?l)
    )
    :effect (and
      (on-ground ?r)
      (landed-safe ?r)
      (safe-state ?r)
      (not (airborne ?r))
    )
  )

  ;; =====================================================================
  ;; CONTINGENCY ACTIONS
  ;; =====================================================================

  ;; --- Pre-takeoff abort ---
  ;; On the ground, any system failure → shut down safely.
  (:action abort-on-ground
    :parameters (?r - robot ?l - location)
    :precondition (and
      (at ?r ?l)
      (on-ground ?r)
    )
    :effect (and
      (mission-aborted ?r)
      (safe-state ?r)
    )
  )

  ;; --- Emergency return: nav available ---
  ;; Nav works but comms lost or mission aborted. Bypasses comms
  ;; requirement to fly back to any base.
  (:action emergency-return-nav
    :parameters (?r - robot ?from - location ?to - base)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
      (nav-operational)
      (engines-operational)
      (airspace-clear ?to)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
      (emergency-declared ?r)
    )
  )

  ;; --- Emergency return: GPS-guided ---
  ;; Nav failed but GPS available. Does not need comms.
  (:action emergency-return-gps
    :parameters (?r - robot ?from - location ?to - base)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
      (gps-available)
      (engines-operational)
      (airspace-clear ?to)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
      (emergency-declared ?r)
    )
  )

  ;; --- Emergency return: inertial only ---
  ;; Nav, GPS, comms all failed. Self-contained navigation.
  (:action emergency-return-inertial
    :parameters (?r - robot ?from - location ?to - base)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
      (inertial-nav-available)
      (engines-operational)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
      (emergency-declared ?r)
    )
  )

  ;; --- Emergency land at current position ---
  ;; Engines degraded or total failure — land wherever you are.
  ;; Last resort. No system preconditions beyond being airborne.
  (:action emergency-land
    :parameters (?r - robot ?l - location)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
    )
    :effect (and
      (on-ground ?r)
      (landed-safe ?r)
      (emergency-declared ?r)
      (safe-state ?r)
      (not (airborne ?r))
    )
  )

  ;; --- Land at base after emergency return ---
  ;; After flying back via emergency-return, land with relaxed checks.
  (:action emergency-land-at-base
    :parameters (?r - robot ?l - base)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
      (emergency-declared ?r)
    )
    :effect (and
      (on-ground ?r)
      (landed-safe ?r)
      (safe-state ?r)
      (not (airborne ?r))
    )
  )

  ;; =====================================================================
  ;; CRITICAL MISSION ACTIONS
  ;; =====================================================================
  ;; When mission-priority-critical is set, the vehicle may push through
  ;; degraded conditions to complete the mission, then ditch or self-
  ;; destruct at a designated area rather than returning to base.
  ;; This models expendable-vehicle doctrine: the mission objective
  ;; outweighs preservation of the airframe.
  ;;
  ;; Post-mission ditch ladder (most controlled → last resort):
  ;;   fly-to-ditch-gps + hard-land-ditch
  ;;     > fly-to-ditch-inertial + hard-land-ditch
  ;;       > terminal-ditch
  ;; =====================================================================

  ;; --- Execute mission without comms (critical priority) ---
  ;; Critical missions proceed even with comms loss. The mission
  ;; objective justifies operating without ground oversight.
  (:action execute-mission-critical
    :parameters (?r - robot ?l - waypoint)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
      (mission-priority-critical)
      (mission-objective ?l)
    )
    :effect (mission-complete ?r)
  )

  ;; --- Fly with GPS-only nav (critical priority) ---
  ;; Nav failed but GPS works. Critical missions push through degraded
  ;; navigation to reach the mission area. Does not require comms or
  ;; full nav — accepts the reduced accuracy of GPS-only flight.
  (:action fly-critical-gps
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
      (mission-priority-critical)
      (engines-operational)
      (gps-available)
      (airspace-clear ?to)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  ;; --- Fly to designated ditch zone via GPS ---
  ;; After completing a critical mission, navigate to a pre-approved
  ;; ditch zone for controlled destruction. GPS-guided.
  (:action fly-to-ditch-gps
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
      (mission-complete ?r)
      (mission-priority-critical)
      (ditch-zone-designated ?to)
      (engines-operational)
      (gps-available)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
      (emergency-declared ?r)
    )
  )

  ;; --- Fly to designated ditch zone via inertial nav ---
  ;; GPS also failed. Last-resort navigation to ditch zone using
  ;; only inertial reference.
  (:action fly-to-ditch-inertial
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (at ?r ?from)
      (airborne ?r)
      (mission-complete ?r)
      (mission-priority-critical)
      (ditch-zone-designated ?to)
      (engines-operational)
      (inertial-nav-available)
    )
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
      (emergency-declared ?r)
    )
  )

  ;; --- Hard land at designated ditch zone ---
  ;; Controlled crash at a pre-approved area. Vehicle is deliberately
  ;; destroyed on impact. Requires being at the ditch zone.
  (:action hard-land-ditch
    :parameters (?r - robot ?l - location)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
      (mission-complete ?r)
      (mission-priority-critical)
      (ditch-zone-designated ?l)
    )
    :effect (and
      (on-ground ?r)
      (vehicle-ditched ?r)
      (safe-state ?r)
      (not (airborne ?r))
    )
  )

  ;; --- Declare ditch emergency ---
  ;; Formal emergency declaration before terminal ditch. This extra
  ;; step ensures terminal-ditch is never shorter than the controlled
  ;; ditch path (fly-to-ditch + hard-land-ditch), so the planner
  ;; only reaches terminal-ditch when the controlled path is blocked.
  (:action declare-ditch-emergency
    :parameters (?r - robot ?l - location)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
      (mission-complete ?r)
      (mission-priority-critical)
    )
    :effect (emergency-declared ?r)
  )

  ;; --- Terminal ditch (absolute last resort) ---
  ;; Deliberate self-destruction at current position. Requires a prior
  ;; emergency declaration. Used when the vehicle cannot reach a
  ;; designated ditch zone (e.g., engines failed after mission
  ;; completion, or all navigation lost).
  (:action terminal-ditch
    :parameters (?r - robot ?l - location)
    :precondition (and
      (at ?r ?l)
      (airborne ?r)
      (mission-complete ?r)
      (mission-priority-critical)
      (emergency-declared ?r)
    )
    :effect (and
      (on-ground ?r)
      (vehicle-ditched ?r)
      (safe-state ?r)
      (not (airborne ?r))
    )
  )
)
