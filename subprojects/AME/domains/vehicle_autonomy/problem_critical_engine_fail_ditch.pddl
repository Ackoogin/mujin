;;; ==========================================================================
;;; Vehicle Autonomy — Critical mission: engine failure post-mission, ditch
;;; ==========================================================================
;;; Critical mission already completed at wp1. Engines failed after mission
;;; completion. All other systems operational but the vehicle cannot fly
;;; to the designated ditch zone. Falls through to terminal-ditch (last-
;;; resort self-destruction at current position).
;;;
;;; Expected: SOLVABLE via terminal-ditch (1 step).
;;; Even with nav/GPS working, no engines means no flight to dz1.

(define (problem vehicle-critical-engine-fail-ditch)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
    dz1 - waypoint
  )

  (:init
    (at uav1 wp1)
    (airborne uav1)
    (preflight-done uav1)
    (mission-complete uav1)

    (mission-priority-critical)
    (mission-objective wp1)

    (nav-operational)
    ;; FAULT: engines-operational ABSENT (failed after mission completion)
    (comms-available)
    (gps-available)
    (inertial-nav-available)

    (airspace-clear home)
    (airspace-clear wp1)
    (airspace-clear dz1)
    (weather-ok home)
    (weather-ok wp1)
    (weather-ok dz1)

    (ditch-zone-designated dz1)
  )

  (:goal (and
    (mission-complete uav1)
    (vehicle-ditched uav1)
    (safe-state uav1)
  ))
)
