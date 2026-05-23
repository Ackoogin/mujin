;;; ==========================================================================
;;; Vehicle Autonomy — Engine failure before takeoff
;;; ==========================================================================
;;; Engines failed before takeoff. UAV stays on ground, aborts.
;;; Validates that any pre-takeoff fault has a safe exit path.
;;;
;;; Expected: SOLVABLE via abort-on-ground (1 step).

(define (problem vehicle-engine-fail-pre-takeoff)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
  )

  (:init
    (at uav1 home)
    (on-ground uav1)

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
    (mission-aborted uav1)
  ))
)
