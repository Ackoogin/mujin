;;; ==========================================================================
;;; Vehicle Autonomy — Engine failure while airborne
;;; ==========================================================================
;;; Engines degraded while airborne at wp1. All emergency-return actions
;;; require engines. Only option: emergency-land at current position.
;;;
;;; Expected: SOLVABLE via emergency-land (1 step). UAV lands at wp1,
;;; not at base.

(define (problem vehicle-engine-fail-airborne)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
  )

  (:init
    (at uav1 wp1)
    (airborne uav1)
    (preflight-done uav1)

    (nav-operational)
    ;; FAULT: engines-operational ABSENT
    (comms-available)
    (gps-available)
    (inertial-nav-available)

    (airspace-clear home)
    (airspace-clear wp1)
    (weather-ok home)
    (weather-ok wp1)
  )

  (:goal (and
    (safe-state uav1)
    (landed-safe uav1)
  ))
)
