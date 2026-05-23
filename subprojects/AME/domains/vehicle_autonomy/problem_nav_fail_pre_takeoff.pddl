;;; ==========================================================================
;;; Vehicle Autonomy — Nav failure before takeoff
;;; ==========================================================================
;;; Nav failed during preflight. UAV is on the ground. Cannot take off.
;;; Must abort safely on the ground.
;;;
;;; Expected: SOLVABLE via abort-on-ground (1 step).

(define (problem vehicle-nav-fail-pre-takeoff)
  (:domain vehicle-autonomy)

  (:objects
    uav1 - robot
    home - base
    wp1 - waypoint
  )

  (:init
    (at uav1 home)
    (on-ground uav1)

    ;; FAULT: nav-operational ABSENT
    (engines-operational)
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
