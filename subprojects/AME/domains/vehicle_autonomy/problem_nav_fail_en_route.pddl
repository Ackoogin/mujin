;;; ==========================================================================
;;; Vehicle Autonomy — Nav failure en route
;;; ==========================================================================
;;; UAV is airborne at wp1 when nav fails. Cannot use nominal fly.
;;; GPS and inertial still available for emergency return.
;;;
;;; Expected: SOLVABLE via emergency-return-gps + emergency-land-at-base
;;; (or emergency-return-inertial).

(define (problem vehicle-nav-fail-en-route)
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
    (landed-safe uav1)
    (at uav1 home)
  ))
)
